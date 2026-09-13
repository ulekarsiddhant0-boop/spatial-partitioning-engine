#include "raylib.h"
#include "Quadtree.hpp"
#include <vector>
#include <random>
#include <chrono>

const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 800;
const int ENTITY_COUNT = 2500;

void resolveCollision(Entity &a, Entity &b)
{
    Vec2 delta = b.pos - a.pos;
    float distSq = delta.lengthSq();
    float radSum = a.radius + b.radius;

    if (distSq > 0.0f && distSq <= (radSum * radSum))
    {
        a.isColliding = true;
        b.isColliding = true;

        float dist = std::sqrt(distSq);
        Vec2 normal = delta * (1.0f / dist);

        // Positional separation (prevent sinking)
        float overlap = 0.5f * (radSum - dist);
        a.pos = a.pos - normal * overlap;
        b.pos = b.pos + normal * overlap;

        // Elastic momentum resolution
        Vec2 relVel = b.vel - a.vel;
        float velAlongNormal = relVel.dot(normal);

        if (velAlongNormal < 0.0f)
        {                              // Moving towards each other
            float restitution = 0.85f; // Bouncy
            float impulseScalar = -(1.0f + restitution) * velAlongNormal / (1.0f / a.mass + 1.0f / b.mass);
            Vec2 impulse = normal * impulseScalar;

            a.vel = a.vel - impulse * (1.0f / a.mass);
            b.vel = b.vel + impulse * (1.0f / b.mass);
        }
    }
}

int main()
{
    SetConfigFlags(FLAG_VSYNC_HINT);
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Spatial Partitioning Engine - EA Slingshot Prototype");
    SetTargetFPS(144);

    std::mt19937 rng(1337);
    std::uniform_real_distribution<float> posX(30.0f, SCREEN_WIDTH - 30.0f);
    std::uniform_real_distribution<float> posY(30.0f, SCREEN_HEIGHT - 30.0f);
    std::uniform_real_distribution<float> speed(-120.0f, 120.0f);
    std::uniform_real_distribution<float> radiusDist(2.5f, 5.0f);

    std::vector<Entity> entities;
    entities.reserve(ENTITY_COUNT);

    for (int i = 0; i < ENTITY_COUNT; ++i)
    {
        float r = radiusDist(rng);
        entities.push_back({i,
                            {posX(rng), posY(rng)},
                            {speed(rng), speed(rng)},
                            r,
                            r * 0.5f, // Mass proportional to radius
                            false});
    }

    QuadtreePool tree;
    std::vector<int> candidates;
    candidates.reserve(64);

    bool useQuadtree = true;
    bool drawGrid = true;
    double execTimeUs = 0.0;
    int collisionChecks = 0;

    AABB mouseInspectionArea{-100, -100, 60, 60};

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        if (dt > 0.033f)
            dt = 0.033f; // Clamp to avoid spiral of death

        if (IsKeyPressed(KEY_SPACE))
            useQuadtree = !useQuadtree;
        if (IsKeyPressed(KEY_T))
            drawGrid = !drawGrid;

        // Mouse Query Inspector
        Vector2 mousePos = GetMousePosition();
        mouseInspectionArea.x = mousePos.x;
        mouseInspectionArea.y = mousePos.y;

        // 1. Move Entities & Wall Collisions
        for (auto &e : entities)
        {
            e.isColliding = false;
            e.pos = e.pos + e.vel * dt;

            if (e.pos.x - e.radius < 0)
            {
                e.pos.x = e.radius;
                e.vel.x *= -1;
            }
            if (e.pos.x + e.radius > SCREEN_WIDTH)
            {
                e.pos.x = SCREEN_WIDTH - e.radius;
                e.vel.x *= -1;
            }
            if (e.pos.y - e.radius < 0)
            {
                e.pos.y = e.radius;
                e.vel.y *= -1;
            }
            if (e.pos.y + e.radius > SCREEN_HEIGHT)
            {
                e.pos.y = SCREEN_HEIGHT - e.radius;
                e.vel.y *= -1;
            }
        }

        collisionChecks = 0;
        auto start = std::chrono::high_resolution_clock::now();

        // 2. Collision Phase
        if (!useQuadtree)
        {
            // O(N^2) Brute Force
            for (size_t i = 0; i < entities.size(); ++i)
            {
                for (size_t j = i + 1; j < entities.size(); ++j)
                {
                    collisionChecks++;
                    resolveCollision(entities[i], entities[j]);
                }
            }
        }
        else
        {
            // O(N log N) Linear Pool Quadtree
            AABB bounds{SCREEN_WIDTH * 0.5f, SCREEN_HEIGHT * 0.5f, SCREEN_WIDTH * 0.5f, SCREEN_HEIGHT * 0.5f};
            tree.reset(bounds, entities);

            for (size_t i = 0; i < entities.size(); ++i)
            {
                tree.insert(static_cast<int>(i));
            }

            for (size_t i = 0; i < entities.size(); ++i)
            {
                candidates.clear();
                AABB searchArea{entities[i].pos.x, entities[i].pos.y, entities[i].radius * 2, entities[i].radius * 2};
                tree.query(searchArea, candidates);

                for (int otherIdx : candidates)
                {
                    if (entities[i].id >= entities[otherIdx].id)
                        continue;
                    collisionChecks++;
                    resolveCollision(entities[i], entities[otherIdx]);
                }
            }
        }

        auto end = std::chrono::high_resolution_clock::now();
        execTimeUs = std::chrono::duration<double, std::micro>(end - start).count();

        // 3. Render
        BeginDrawing();
        ClearBackground({14, 14, 18, 255});

        // Draw Tree Grid Lines
        if (useQuadtree && drawGrid)
        {
            const auto &allNodes = tree.getAllNodes();
            for (const auto &node : allNodes)
            {
                DrawRectangleLines(
                    (int)(node.boundary.x - node.boundary.halfW),
                    (int)(node.boundary.y - node.boundary.halfH),
                    (int)(node.boundary.halfW * 2),
                    (int)(node.boundary.halfH * 2),
                    Fade(SKYBLUE, 0.15f));
            }
        }

        // Draw Entities
        for (const auto &e : entities)
        {
            DrawCircle((int)e.pos.x, (int)e.pos.y, e.radius, e.isColliding ? Color{235, 75, 75, 255} : Color{70, 205, 120, 255});
        }

        // Draw Mouse Range Inspector
        DrawRectangleLines(
            (int)(mouseInspectionArea.x - mouseInspectionArea.halfW),
            (int)(mouseInspectionArea.y - mouseInspectionArea.halfH),
            (int)(mouseInspectionArea.halfW * 2),
            (int)(mouseInspectionArea.halfH * 2),
            YELLOW);
        std::vector<int> inspected;
        if (useQuadtree)
        {
            tree.query(mouseInspectionArea, inspected);
            for (int idx : inspected)
            {
                DrawCircleLines((int)entities[idx].pos.x, (int)entities[idx].pos.y, entities[idx].radius + 3.0f, YELLOW);
            }
        }

        // UI Dashboard
        DrawRectangle(15, 15, 450, 165, Fade(BLACK, 0.85f));
        DrawRectangleLines(15, 15, 450, 165, DARKGRAY);

        DrawText("ENGINE ARCHITECTURE BENCHMARK", 25, 25, 14, RAYWHITE);
        DrawText(TextFormat("Mode: %s [SPACE to Toggle]", useQuadtree ? "Contiguous Pool Quadtree" : "Brute Force O(N^2)"), 25, 45, 16, useQuadtree ? GREEN : RED);
        DrawText(TextFormat("FPS: %i", GetFPS()), 25, 70, 20, RAYWHITE);
        DrawText(TextFormat("Active Entities: %i", (int)entities.size()), 25, 95, 16, RAYWHITE);
        DrawText(TextFormat("Broadphase Checks: %i", collisionChecks), 25, 118, 16, ORANGE);
        DrawText(TextFormat("Frame Step Time: %.2f ms (%.0f us)", execTimeUs / 1000.0, execTimeUs), 25, 140, 16, SKYBLUE);
        DrawText(TextFormat("Hover Query Inspected: %i entities", (int)inspected.size()), 25, 160, 12, YELLOW);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}