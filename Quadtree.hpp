#pragma once
#include <vector>
#include <cmath>
#include <cstdint>

struct Vec2
{
    float x, y;

    Vec2 operator+(const Vec2 &o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2 &o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(float s) const { return {x * s, y * s}; }
    float dot(const Vec2 &o) const { return x * o.x + y * o.y; }
    float lengthSq() const { return x * x + y * y; }
    float length() const { return std::sqrt(lengthSq()); }
    Vec2 normalized() const
    {
        float l = length();
        return l > 0.0001f ? Vec2{x / l, y / l} : Vec2{0, 0};
    }
};

struct AABB
{
    float x, y; // Center
    float halfW, halfH;

    bool contains(const Vec2 &pt) const
    {
        return (pt.x >= x - halfW && pt.x <= x + halfW &&
                pt.y >= y - halfH && pt.y <= y + halfH);
    }

    bool intersects(const AABB &o) const
    {
        return !(o.x - o.halfW > x + halfW ||
                 o.x + o.halfW < x - halfW ||
                 o.y - o.halfH > y + halfH ||
                 o.y + o.halfH < y - halfH);
    }
};

struct Entity
{
    int id;
    Vec2 pos;
    Vec2 vel;
    float radius;
    float mass;
    bool isColliding;
};

// Flat array Node Pool Quadtree: Zero runtime heap fragmentation
class QuadtreePool
{
public:
    static constexpr int CAPACITY = 8;
    static constexpr int MAX_DEPTH = 6;

    struct Node
    {
        AABB boundary;
        int depth;
        int count = 0;
        int entityIndices[CAPACITY];
        int childIndex = -1; // Index in node pool; -1 if leaf
        bool isLeaf() const { return childIndex == -1; }
    };

private:
    std::vector<Node> nodes;
    const std::vector<Entity> *entitiesRef;

    void subdivide(int nodeIdx)
    {
        int nextChildIdx = static_cast<int>(nodes.size());
        nodes[nodeIdx].childIndex = nextChildIdx;
        nodes.resize(nextChildIdx + 4);

        float x = nodes[nodeIdx].boundary.x;
        float y = nodes[nodeIdx].boundary.y;
        float w = nodes[nodeIdx].boundary.halfW * 0.5f;
        float h = nodes[nodeIdx].boundary.halfH * 0.5f;
        int d = nodes[nodeIdx].depth + 1;

        nodes[nextChildIdx + 0] = Node{{x - w, y - h, w, h}, d}; // NW
        nodes[nextChildIdx + 1] = Node{{x + w, y - h, w, h}, d}; // NE
        nodes[nextChildIdx + 2] = Node{{x - w, y + h, w, h}, d}; // SW
        nodes[nextChildIdx + 3] = Node{{x + w, y + h, w, h}, d}; // SE

        // Re-distribute parent elements to children
        for (int i = 0; i < nodes[nodeIdx].count; ++i)
        {
            int eIdx = nodes[nodeIdx].entityIndices[i];
            insertIntoChildren(nodeIdx, eIdx);
        }
        nodes[nodeIdx].count = 0;
    }

    bool insertIntoChildren(int parentIdx, int entityIdx)
    {
        int base = nodes[parentIdx].childIndex;
        const Vec2 &p = (*entitiesRef)[entityIdx].pos;

        for (int i = 0; i < 4; ++i)
        {
            if (nodes[base + i].boundary.contains(p))
            {
                return insertInternal(base + i, entityIdx);
            }
        }
        return false;
    }

    bool insertInternal(int nodeIdx, int entityIdx)
    {
        if (!nodes[nodeIdx].isLeaf())
        {
            return insertIntoChildren(nodeIdx, entityIdx);
        }

        if (nodes[nodeIdx].count < CAPACITY || nodes[nodeIdx].depth >= MAX_DEPTH)
        {
            nodes[nodeIdx].entityIndices[nodes[nodeIdx].count++] = entityIdx;
            return true;
        }

        subdivide(nodeIdx);
        return insertIntoChildren(nodeIdx, entityIdx);
    }

public:
    QuadtreePool()
    {
        nodes.reserve(2048);
    }

    void reset(AABB rootBoundary, const std::vector<Entity> &entities)
    {
        nodes.clear();
        entitiesRef = &entities;
        nodes.push_back(Node{rootBoundary, 0});
    }

    void insert(int entityIdx)
    {
        insertInternal(0, entityIdx);
    }

    void query(const AABB &range, std::vector<int> &results, int nodeIdx = 0) const
    {
        if (nodeIdx < 0 || nodeIdx >= (int)nodes.size())
            return;
        const auto &node = nodes[nodeIdx];

        if (!node.boundary.intersects(range))
            return;

        for (int i = 0; i < node.count; ++i)
        {
            int eIdx = node.entityIndices[i];
            if (range.contains((*entitiesRef)[eIdx].pos))
            {
                results.push_back(eIdx);
            }
        }

        if (!node.isLeaf())
        {
            for (int i = 0; i < 4; ++i)
            {
                query(range, results, node.childIndex + i);
            }
        }
    }

    const std::vector<Node> &getAllNodes() const { return nodes; }
};