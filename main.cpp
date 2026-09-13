#include "raylib.h"
#include "Quadtree.hpp"
#include <vector>
#include <random>
#include <chrono>
#include <string>

const int SCREEN_WIDTH = 1200;
const int SCREEN_HEIGHT = 800;
const int ENTITY_COUNT = 2000;

bool checkCollision(const Entity &a, const Entity &b)
{
    float dx = a.pos.x - b.pos.x;
    float dy = a.pos.y - b.pos.y;
    float distSq = dx * dx + dy * dy;
    float radSum = a.radius + b.radius;
    return distSq <= (radSum * radSum);
}

int main()
{
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Spatial Partitioning Engine Benchmark");
    SetTargetFPS(120);

    // Initialize random entities
    // Initialize random entities
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> distX(20.0f, SCREEN_WIDTH - 20.0f);
    std::uniform_real_distribution<float> distY(20.0f, SCREEN_HEIGHT - 20.0f);
    std::uniform_real_distribution<float> distVel(-100.0f, 100.0f);

    std::vector<Entity> entities;
    entities.reserve(ENTITY_COUNT);

    for (int i = 0; i < ENTITY_COUNT; ++i)
    {
        entities.push_back({i,
                            {distX(rng), distY(rng)},
                            {distVel(rng), distVel(rng)},
                            3.0f,
                            false});
    }

    bool useQuadtree = true;
    bool drawTreeGrid = true;
    double queryDurationMicroseconds = 0.0;
    int collisionChecks = 0;

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();

        // Controls
        if (IsKeyPressed(KEY_SPACE))
            useQuadtree = !useQuadtree;
        if (IsKeyPressed(KEY_T))
            drawTreeGrid = !drawTreeGrid;

        // 1. Update Entity Physics
        for (auto &e : entities)
        {
            e.isColliding = false;
            e.pos.x += e.vel.x * dt;
            e.pos.y += e.vel.y * dt;

            // Bounce off boundaries
            if (e.pos.x - e.radius < 0 || e.pos.x + e.radius > SCREEN_WIDTH)
                e.vel.x *= -1;
            if (e.pos.y - e.radius < 0 || e.pos.y + e.radius > SCREEN_HEIGHT)
                e.vel.y *= -1;
        }

        collisionChecks = 0;
        auto startTime = std::chrono::high_resolution_clock::now();

        // 2. Collision Detection Phase
        if (!useQuadtree)
        {
            // Brute Force: O(N^2)
            for (size_t i = 0; i < entities.size(); ++i)
            {
                for (size_t j = i + 1; j < entities.size(); ++j)
                {
                    collisionChecks++;
                    if (checkCollision(entities[i], entities[j]))
                    {
                        entities[i].isColliding = true;
                        entities[j].isColliding = true;
                    }
                }
            }
        }
        else
        {
            // Spatial Partitioning: O(N log N)
            AABB globalBounds{SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f, SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f};
            Quadtree tree(globalBounds);

            for (auto &e : entities)
            {
                tree.insert(&e);
            }

            std::vector<Entity *> candidates;
            candidates.reserve(32);

            for (auto &e : entities)
            {
                candidates.clear();
                // Query only immediate bounding box of entity
                AABB queryArea{e.pos.x, e.pos.y, e.radius * 2, e.radius * 2};
                tree.query(queryArea, candidates);

                for (Entity *other : candidates)
                {
                    if (e.id >= other->id)
                        continue; // Avoid redundant checks
                    collisionChecks++;
                    if (checkCollision(e, *other))
                    {
                        e.isColliding = true;
                        other->isColliding = true;
                    }
                }
            }
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        queryDurationMicroseconds = std::chrono::duration<double, std::micro>(endTime - startTime).count();

        // 3. Render Phase
        BeginDrawing();
        ClearBackground(BLACK);

        // Draw Quadtree boundaries
        if (useQuadtree && drawTreeGrid)
        {
            AABB globalBounds{SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f, SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f};
            Quadtree debugTree(globalBounds);
            for (auto &e : entities)
                debugTree.insert(&e);

            std::vector<AABB> boxes;
            debugTree.getBoundaries(boxes);
            for (const auto &b : boxes)
            {
                DrawRectangleLines(
                    (int)(b.x - b.halfW),
                    (int)(b.y - b.halfH),
                    (int)(b.halfW * 2),
                    (int)(b.halfH * 2),
                    Fade(DARKGRAY, 0.4f));
            }
        }

        // Draw entities
        for (const auto &e : entities)
        {
            DrawCircle((int)e.pos.x, (int)e.pos.y, e.radius, e.isColliding ? RED : GREEN);
        }

        // Dashboard Overlay
        DrawRectangle(10, 10, 420, 140, Fade(BLACK, 0.75f));
        DrawRectangleLines(10, 10, 420, 140, WHITE);

        DrawText(TextFormat("Mode: %s (Press SPACE to toggle)", useQuadtree ? "Quadtree [O(N log N)]" : "Brute Force [O(N^2)]"), 20, 20, 18, useQuadtree ? GREEN : RED);
        DrawText(TextFormat("FPS: %i", GetFPS()), 20, 45, 20, RAYWHITE);
        DrawText(TextFormat("Entities: %i", ENTITY_COUNT), 20, 70, 18, RAYWHITE);
        DrawText(TextFormat("Collision Checks: %i", collisionChecks), 20, 95, 18, YELLOW);
        DrawText(TextFormat("Engine Time: %.2f ms (%.0f us)", queryDurationMicroseconds / 1000.0, queryDurationMicroseconds), 20, 120, 18, SKYBLUE);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}