#pragma once
#include <vector>
#include <memory>
#include <cmath>

struct Vec2 {
    float x, y;
};

struct AABB {
    float x, y; // Center coordinates
    float halfW, halfH;

    bool contains(const Vec2& point) const {
        return (point.x >= x - halfW && point.x <= x + halfW &&
                point.y >= y - halfH && point.y <= y + halfH);
    }

    bool intersects(const AABB& other) const {
        return !(other.x - other.halfW > x + halfW ||
                 other.x + other.halfW < x - halfW ||
                 other.y - other.halfH > y + halfH ||
                 other.y + other.halfH < y - halfH);
    }
};

struct Entity {
    int id;
    Vec2 pos;
    Vec2 vel;
    float radius;
    bool isColliding;
};

class Quadtree {
private:
    static constexpr int CAPACITY = 8;
    static constexpr int MAX_DEPTH = 6;

    int depth;
    AABB boundary;
    std::vector<Entity*> points;
    bool divided;

    std::unique_ptr<Quadtree> northWest;
    std::unique_ptr<Quadtree> northEast;
    std::unique_ptr<Quadtree> southWest;
    std::unique_ptr<Quadtree> southEast;

    void subdivide() {
        float x = boundary.x;
        float y = boundary.y;
        float w = boundary.halfW / 2.0f;
        float h = boundary.halfH / 2.0f;

        northWest = std::make_unique<Quadtree>(AABB{x - w, y - h, w, h}, depth + 1);
        northEast = std::make_unique<Quadtree>(AABB{x + w, y - h, w, h}, depth + 1);
        southWest = std::make_unique<Quadtree>(AABB{x - w, y + h, w, h}, depth + 1);
        southEast = std::make_unique<Quadtree>(AABB{x + w, y + h, w, h}, depth + 1);

        divided = true;

        // Push existing points into children
        for (Entity* e : points) {
            insertIntoChildren(e);
        }
        points.clear();
    }

    bool insertIntoChildren(Entity* entity) {
        if (northWest->insert(entity)) return true;
        if (northEast->insert(entity)) return true;
        if (southWest->insert(entity)) return true;
        if (southEast->insert(entity)) return true;
        return false;
    }

public:
    Quadtree(AABB boundary, int depth = 0)
        : boundary(boundary), depth(depth), divided(false) {
        points.reserve(CAPACITY);
    }

    bool insert(Entity* entity) {
        if (!boundary.contains(entity->pos)) {
            return false;
        }

        if (!divided) {
            if (points.size() < CAPACITY || depth >= MAX_DEPTH) {
                points.push_back(entity);
                return true;
            }
            subdivide();
        }

        return insertIntoChildren(entity);
    }

    void query(const AABB& range, std::vector<Entity*>& found) const {
        if (!boundary.intersects(range)) {
            return;
        }

        for (Entity* e : points) {
            if (range.contains(e->pos)) {
                found.push_back(e);
            }
        }

        if (divided) {
            northWest->query(range, found);
            northEast->query(range, found);
            southWest->query(range, found);
            southEast->query(range, found);
        }
    }

    // Accessor for rendering tree lines
    void getBoundaries(std::vector<AABB>& boxes) const {
        boxes.push_back(boundary);
        if (divided) {
            northWest->getBoundaries(boxes);
            northEast->getBoundaries(boxes);
            southWest->getBoundaries(boxes);
            southEast->getBoundaries(boxes);
        }
    }
};