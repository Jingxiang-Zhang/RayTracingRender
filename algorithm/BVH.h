#pragma once
#ifndef _BVH_H_
#define _BVH_H_
#include <memory>
#include "../model/scene.h"
#define BOUNDING_BOX_MIN_TRIANGLE 3 // when triangle less than this value, stop dividing

// you can call the following function

namespace SAHHybrid {
	struct BoundingBox {
		double x_min, x_max;
		double y_min, y_max;
		double z_min, z_max;
		BoundingBox() {
			x_min = INT_MAX; x_max = INT_MIN;
			y_min = INT_MAX; y_max = INT_MIN;
			z_min = INT_MAX; z_max = INT_MIN;
		};
		BoundingBox(float x1, float x2, float y1, float y2, float z1, float z2) {
			this->x_min = x1 < x2 ? x1 : x2; this->x_max = x1 > x2 ? x1 : x2;
			this->y_min = y1 < y2 ? y1 : y2; this->y_max = y1 > y2 ? y1 : y2;
			this->z_min = z1 < z2 ? z1 : z2; this->z_max = z1 > z2 ? z1 : z2;
		}
	};

	// union (merge) with two objects
	BoundingBox Union(BoundingBox& box, double point[3]);

	BoundingBox Union(BoundingBox& box1, BoundingBox& box2);

	BoundingBox Union(BoundingBox& box, Object& obj);

	struct LinearBVHNode {
		BoundingBox bounds;
		union {
			int primitivesOffset;    // leaf
			int secondChildOffset;   // interior
		};
		uint16_t nPrimitives;  // 0 -> interior node
		uint16_t pad[1];          // ensure 32 byte total size
	};

	typedef std::vector<LinearBVHNode> LinearBVHMemory;

	struct HybridBVHNode {
		HybridBVHNode* lchild, * rchild;
		BoundingBox boundingbox;
		// int splitAxis; // -1 when child
		int firstObjOffset;
		int nObject;
		void InitLeaf(int first, int n, const BoundingBox& b) {
			firstObjOffset = first;
			nObject = n;
			boundingbox = b;
			lchild = rchild = nullptr;
		}
		void InitInterior(HybridBVHNode* c0, HybridBVHNode* c1) {
			firstObjOffset = -1;
			nObject = 0;
			lchild = c0;
			rchild = c1;
			boundingbox = Union(c0->boundingbox, c1->boundingbox);
			// splitAxis = axis;
		}
	};

	LinearBVHMemory BVHBuildByhybridSAH(objectIndex& indices);

	void BVHShow(HybridBVHNode* BVHRoot, objectIndex indices, int maxdepth = 100000,
		int depth = 0);

	void BVHBuildByhybridSAH(HybridBVHNode*& root, objectIndex& indices);

	struct MortonObj {
		unsigned int index;
		unsigned int mortonCode;
	};

	struct LBVHTreelet {
		int startIndex, nIndex;
		HybridBVHNode* buildNodes;
	};

	struct BucketInfo {
		int count = 0;
		BoundingBox bound;
	};

	inline uint32_t LeftShift3(uint32_t x);

	inline uint32_t EncodeMorton3(double offset[3], int mortonScale);

	inline void obj_centroid(Object& obj, double position[3]);

	void RadixSort(std::vector<MortonObj>* v);

	void emitLBVH(HybridBVHNode* buildNodes, MortonObj* mortonPrims,
		int nObject, int* totalNodes, objectIndex& orderedPrims,
		std::atomic<int>* orderedTrisOffset, int bitIndex);

	void HybridFinalSAH(HybridBVHNode*& self, std::vector<LBVHTreelet>& prims, 
		objectIndex& indices);

	inline int BucketLoc(BoundingBox& bound, BoundingBox& prim, int dim, int nBucket);

	inline double surfaceArea(BoundingBox& box);

	int flattenBVHTree(HybridBVHNode* root, LinearBVHMemory& out, int* offset);

	int countTreeNodes(HybridBVHNode* root);

	void BVHRemove(HybridBVHNode*); // remove the tree

	inline void BVHShowPrintLine(int depth, char c = '-'); // called by BVHShow
}

#endif