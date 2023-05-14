#include "BVH.h"
#include <algorithm>
#include <iostream>
#include "parallel.h"


SAHHybrid::LinearBVHMemory SAHHybrid::BVHBuildByhybridSAH(objectIndex& indices) {
	// build BVH by Hybrid and Surface Area Heuristic
	HybridBVHNode* HybridBVHRoot = new HybridBVHNode;
	BVHBuildByhybridSAH(HybridBVHRoot, indices);
	// BVHShow(HybridBVHRoot, indices);
	LinearBVHMemory out(countTreeNodes(HybridBVHRoot));
	int i = 0;
	flattenBVHTree(HybridBVHRoot, out, &i);
	//for (auto i : out) {
	//	if (i.nPrimitives != 0) {
	//		std::cout << i.primitivesOffset << " " << i.nPrimitives << std::endl;
	//	}
	//	else {
	//		std::cout << i.secondChildOffset << std::endl;
	//	}
	//}
	// BVHShow(HybridBVHRoot, indices);
	BVHRemove(HybridBVHRoot);
	return out;
}


void SAHHybrid::BVHBuildByhybridSAH(HybridBVHNode*& root, objectIndex& indices) {
	// compute the total bounding box
	BoundingBox bound;
	for (Object& obj : obj_list) 
		bound = Union(bound, obj);
	// Compute Morton code for each triangles
	std::vector<MortonObj> mortonTris(obj_list.size());
	// Compute Morton indices of primitives
	parallelRun([&](int i) {
		constexpr int mortonBits = 10;
		constexpr int mortonScale = 1 << mortonBits;
		double centroid[3];
		obj_centroid(obj_list[i], centroid);
		double offset[3] = {
			(centroid[0] - bound.x_min) / (bound.x_max - bound.x_min),
			(centroid[1] - bound.y_min) / (bound.y_max - bound.y_min),
			(centroid[2] - bound.z_min) / (bound.z_max - bound.z_min)
		};
		mortonTris[i].index = i;
		mortonTris[i].mortonCode = SAHHybrid::EncodeMorton3(offset, mortonScale);
	}, obj_list.size());
	// Radix sort primitive Morton indices
	SAHHybrid::RadixSort(&mortonTris);

	// Create LBVH treelets at bottom of BVH
	// Find intervals of primitives for each treelet
	std::vector<LBVHTreelet> treeletsToBuild;
	for (int start = 0, end = 1; end <= (int)mortonTris.size(); ++end) {
		uint32_t mask = 0b00111111111111000000000000000000;
		if (end == (int)mortonTris.size() ||
			((mortonTris[start].mortonCode & mask) !=
				(mortonTris[end].mortonCode & mask))) {
			// Add entry to treeletsToBuild for this treelet 
			int nTriangles = end - start;
			// int maxBVHNodes = 2 * nTriangles - 1;
			HybridBVHNode* nodes = new HybridBVHNode;
			treeletsToBuild.push_back({ start, nTriangles, nodes });
			start = end;
		}
	}
	// Create LBVHs for treelets in parallel
	std::atomic<int> atomicTotal(0), orderedPrimsOffset(0);
	parallelRun([&](int i) {
		int nodesCreated = 0;
		const int firstBitIndex = 29 - 12;
		LBVHTreelet& tr = treeletsToBuild[i];
		emitLBVH(tr.buildNodes, &mortonTris[tr.startIndex], tr.nIndex, &nodesCreated, 
			indices, &orderedPrimsOffset, firstBitIndex);
		atomicTotal += nodesCreated;
	}, treeletsToBuild.size());

	// build the remaining BVH by SAH
	std::vector<unsigned int> indexList(treeletsToBuild.size());
	for (unsigned int i = 0; i < indexList.size(); i++) {
		indexList[i] = i;
	}
	SAHHybrid::HybridFinalSAH(root, treeletsToBuild, indexList);
}

SAHHybrid::BoundingBox SAHHybrid::Union(BoundingBox& box1, double point[3]) {
	BoundingBox box;
	box.x_min = std::min(box1.x_min, point[0]);
	box.y_min = std::min(box1.y_min, point[1]);
	box.z_min = std::min(box1.z_min, point[2]);
	box.x_max = std::max(box1.x_max, point[0]);
	box.y_max = std::max(box1.y_max, point[1]);
	box.z_max = std::max(box1.z_max, point[2]);
	return box;
}

SAHHybrid::BoundingBox SAHHybrid::Union(BoundingBox& box1, BoundingBox& box2) {
	BoundingBox box;
	box.x_min = std::min(box1.x_min, box2.x_min);
	box.y_min = std::min(box1.y_min, box2.y_min);
	box.z_min = std::min(box1.z_min, box2.z_min);
	box.x_max = std::max(box1.x_max, box2.x_max);
	box.y_max = std::max(box1.y_max, box2.y_max);
	box.z_max = std::max(box1.z_max, box2.z_max);
	return box;
}
	
SAHHybrid::BoundingBox SAHHybrid::Union(BoundingBox& box1, Object& obj) {
	BoundingBox box = box1;
	if (obj.type == Object::objType::triangle) {
		Triangle& triangle = *obj.pointer.triangle;
		for (unsigned int i = 0; i < 3; i++) {
			box.x_min = std::min(box.x_min, triangle.v[i].position[0]);
			box.y_min = std::min(box.y_min, triangle.v[i].position[1]);
			box.z_min = std::min(box.z_min, triangle.v[i].position[2]);
			box.x_max = std::max(box.x_max, triangle.v[i].position[0]);
			box.y_max = std::max(box.y_max, triangle.v[i].position[1]);
			box.z_max = std::max(box.z_max, triangle.v[i].position[2]);
		}
	}
	else if (obj.type == Object::objType::sphere) {
		Sphere& sphere = *obj.pointer.sphere;
		box.x_min = std::min(box1.x_min, sphere.position[0] - sphere.radius);
		box.y_min = std::min(box1.y_min, sphere.position[1] - sphere.radius);
		box.z_min = std::min(box1.z_min, sphere.position[2] - sphere.radius);
		box.x_max = std::max(box1.x_max, sphere.position[0] + sphere.radius);
		box.y_max = std::max(box1.y_max, sphere.position[1] + sphere.radius);
		box.z_max = std::max(box1.z_max, sphere.position[2] + sphere.radius);
	}
	return box;
}

uint32_t SAHHybrid::LeftShift3(uint32_t x)
{
	if (x == (1 << 10)) --x;
	x = (x | (x << 16)) & 0b00000011000000000000000011111111;
	x = (x | (x << 8)) & 0b00000011000000001111000000001111;
	x = (x | (x << 4)) & 0b00000011000011000011000011000011;
	x = (x | (x << 2)) & 0b00001001001001001001001001001001;
	return x;
}

uint32_t SAHHybrid::EncodeMorton3(double offset[3], int mortonScale)
{
	return (LeftShift3((uint32_t)(offset[2] * mortonScale)) << 2) |
		(LeftShift3((uint32_t)(offset[1] * mortonScale)) << 1) |
		LeftShift3((uint32_t)(offset[0] * mortonScale));
}

void SAHHybrid::obj_centroid(Object& obj, double position[3]) {
	if (obj.type == Object::objType::sphere) {
		memcpy(position, obj.pointer.sphere->position, sizeof(double) * 3);
	}
	else if (obj.type == Object::objType::triangle) {
		auto& vertices = obj.pointer.triangle->v;
		position[0] = (vertices[0].position[0] + vertices[1].position[0] +
			vertices[2].position[0]) / 3;
		position[1] = (vertices[0].position[1] + vertices[1].position[1] +
			vertices[2].position[1]) / 3;
		position[2] = (vertices[0].position[2] + vertices[1].position[2] +
			vertices[2].position[2]) / 3;
	}
}

void SAHHybrid::RadixSort(std::vector<MortonObj>* v) {
	std::vector<MortonObj> tempVector(v->size());
	constexpr int bitsPerPass = 6;
	constexpr int nBits = 30;
	constexpr int nPasses = nBits / bitsPerPass;
	for (int pass = 0; pass < nPasses; ++pass) {
		int lowBit = pass * bitsPerPass;
		// Set in and out vector pointers for radix sort pass
		std::vector<MortonObj>& in = (pass & 1) ? tempVector : *v;
		std::vector<MortonObj>& out = (pass & 1) ? *v : tempVector;
		// Count number of zero bits in array for current radix sort bit
		constexpr int nBuckets = 1 << bitsPerPass;
		int bucketCount[nBuckets] = { 0 };
		constexpr int bitMask = (1 << bitsPerPass) - 1;
		for (const MortonObj& mp : in) {
			int bucket = (mp.mortonCode >> lowBit) & bitMask;
			++bucketCount[bucket];
		}
		// Compute starting index in output array for each bucket
		int outIndex[nBuckets];
		outIndex[0] = 0;
		for (int i = 1; i < nBuckets; ++i)
			outIndex[i] = outIndex[i - 1] + bucketCount[i - 1];
		// Store sorted values in output array
		for (const MortonObj& mp : in) {
			int bucket = (mp.mortonCode >> lowBit) & bitMask;
			out[outIndex[bucket]++] = mp;
		}

	}
	if (nPasses & 1)
		std::swap(*v, tempVector);
}

void SAHHybrid::emitLBVH(HybridBVHNode* buildNodes, MortonObj* mortonPrims,
	int nObject, int* totalNodes, objectIndex& orderedPrims,
	std::atomic<int>* orderedTrisOffset, int bitIndex) {
	if (bitIndex == -1 || nObject < BOUNDING_BOX_MIN_TRIANGLE) {
		// Create and return leaf node of LBVH treelet
		(*totalNodes)++;
		BoundingBox bound;
		int firstPrimOffset = orderedTrisOffset->fetch_add(nObject);
		for (int i = 0; i < nObject; ++i) {
			int primitiveIndex = mortonPrims[i].index;
			orderedPrims[firstPrimOffset + i] = primitiveIndex;
			bound = Union(bound, obj_list[primitiveIndex]);
		}
		buildNodes->InitLeaf(firstPrimOffset, nObject, bound);
	}
	else {
		int mask = 1 << bitIndex;
		// Advance to next subtree level if there¡¯s no LBVH split for this bit
		if ((mortonPrims[0].mortonCode & mask) ==
			(mortonPrims[nObject - 1].mortonCode & mask))
			return emitLBVH(buildNodes, mortonPrims, nObject, totalNodes, 
				orderedPrims, orderedTrisOffset, bitIndex - 1);
		// Find LBVH split point for this dimension
		int searchStart = 0, searchEnd = nObject - 1;
		while (searchStart + 1 != searchEnd) {
			int mid = (searchStart + searchEnd) / 2;
			if ((mortonPrims[searchStart].mortonCode & mask) ==
				(mortonPrims[mid].mortonCode & mask))
				searchStart = mid;
			else
				searchEnd = mid;
		}
		int splitOffset = searchEnd;
		// Create and return interior LBVH node 
		(*totalNodes)++;
		HybridBVHNode* l_node = new HybridBVHNode;
		emitLBVH(l_node, mortonPrims, splitOffset, totalNodes, orderedPrims, 
			orderedTrisOffset, bitIndex - 1);
		HybridBVHNode* r_node = new HybridBVHNode;
		emitLBVH(r_node, &mortonPrims[splitOffset], nObject - splitOffset, 
			totalNodes, orderedPrims, orderedTrisOffset, bitIndex - 1);
		// int axis = bitIndex % 3;
		buildNodes->InitInterior(l_node, r_node);
	}
}

void SAHHybrid::HybridFinalSAH(HybridBVHNode*& self, std::vector<LBVHTreelet>& prims,
	objectIndex& indices) {
	// std::cout << indices.size() << std::endl;
	if (indices.size() == 0) return;
	if (indices.size() == 1) {
		delete self;
		self = prims[indices[0]].buildNodes;
		return;
	}
	if (indices.size() == 2) {
		// two treelets remain, then one for the left, one for the right
		auto& l_node = prims[indices[0]].buildNodes;
		auto& r_node = prims[indices[1]].buildNodes;
		self->lchild = l_node;
		self->rchild = r_node;
		self->nObject = 0;
		self->boundingbox = Union(l_node->boundingbox, r_node->boundingbox);
		return;
	}

	// Compute bound of primitive centroids, choose split dimension dim
	BoundingBox bound;
	for (unsigned int index : indices) {
		bound = Union(bound, prims[index].buildNodes->boundingbox);
	}
	// test for all three dim
	// define number of buckets
	int nBuckets = 0;
	if (indices.size() < 10) nBuckets = indices.size() / 2 + 1;
	else if (indices.size() < 100) nBuckets = (indices.size() - 10) / 4 + 5;
	else nBuckets = 30;
	// define the final separation
	double minCost = INT_MAX;
	std::vector<std::vector<unsigned int>> indices_in_left;
	std::vector<std::vector<unsigned int>> indices_in_right;
	for (int dim = 0; dim < 3; dim++) {
		BucketInfo* buckets = new BucketInfo[nBuckets];
		// Allocate BucketInfo for SAH partition buckets
		// record primatives index in each bucket 
		std::vector<std::vector<unsigned int>> indices_in_bucket;
		for (int i = 0; i < nBuckets; i++) {
			std::vector<unsigned int> indice_list;
			indices_in_bucket.push_back(indice_list);
		}
		// Initialize BucketInfo for SAH partition buckets
		for (unsigned int index : indices) {
			auto box = prims[index].buildNodes->boundingbox;
			int loc = BucketLoc(bound, box, dim, nBuckets);
			indices_in_bucket[loc].push_back(index);
			buckets[loc].count += prims[index].nIndex;
			buckets[loc].bound = Union(buckets[loc].bound, box);
		}
		// Compute costs for splitting after each bucket
		for (int i = 0; i < nBuckets - 1; ++i) {
			std::vector<std::vector<unsigned int>> indices_in_left_current;
			std::vector<std::vector<unsigned int>> indices_in_right_current;
			BoundingBox b0, b1;
			int count0 = 0, count1 = 0;
			for (int j = 0; j <= i; ++j) {
				b0 = Union(b0, buckets[j].bound);
				count0 += buckets[j].count;
				indices_in_left_current.push_back(indices_in_bucket[j]);
			}
			for (int j = i + 1; j < nBuckets; ++j) {
				b1 = Union(b1, buckets[j].bound);
				count1 += buckets[j].count;
				indices_in_right_current.push_back(indices_in_bucket[j]);
			}
			double boundSurfaceArea = surfaceArea(bound);
			double b0SurfaceArea = surfaceArea(b0);
			double b1SurfaceArea = surfaceArea(b1);
			double cost = 0.125f + (count0 * b0SurfaceArea +
				count1 * b1SurfaceArea) / boundSurfaceArea;
			if (cost < minCost) {
				minCost = cost;
				indices_in_left = indices_in_left_current;
				indices_in_right = indices_in_right_current;
			}
		}
		delete[] buckets;
	}
	// append left and right indices
	std::vector<unsigned int> flat_indices_left;
	std::vector<unsigned int> flat_indices_right;
	{
		for (auto indices : indices_in_left) {
			for (unsigned int number : indices) {
				flat_indices_left.push_back(number);
			}
		}
		for (auto indices : indices_in_right) {
			for (unsigned int number : indices) {
				flat_indices_right.push_back(number);
			}
		}
	}
	// prevent all the node go into one branch
	{
		if (flat_indices_left.size() == 0 || flat_indices_right.size() == 0) {
			std::vector<unsigned int>* zero_number_branch, * full_number_branch;
			if (flat_indices_left.size() == 0) {
				zero_number_branch = &flat_indices_left;
				full_number_branch = &flat_indices_right;
			}
			else {
				zero_number_branch = &flat_indices_right;
				full_number_branch = &flat_indices_left;
			}
			for (unsigned int i = 0; i < full_number_branch->size() / 2; i++) {
				int number = full_number_branch->back();
				full_number_branch->pop_back();
				zero_number_branch->push_back(number);
			}
		}
	}
	// build the tree recursively 
	HybridBVHNode* l_node = new HybridBVHNode;
	HybridBVHNode* r_node = new HybridBVHNode;
	// std::cout << flat_indices_left.size() << " " << flat_indices_right.size() << std::endl;
	HybridFinalSAH(l_node, prims, flat_indices_left);
	HybridFinalSAH(r_node, prims, flat_indices_right);
	self->InitInterior(l_node, r_node);
}

inline int SAHHybrid::BucketLoc(BoundingBox& bound, BoundingBox& prim, int dim, 
	int nBucket) {
	double center = 0.0f;
	double offset = 0.0f;
	switch (dim) {
	case 0:
		center = (prim.x_max + prim.x_min) / 2;
		if (bound.x_max - bound.x_min > 0.00001)
			offset = (center - bound.x_min) / (bound.x_max - bound.x_min);
		else return 0;
		break;
	case 1:
		center = (prim.y_max + prim.y_min) / 2;
		if (bound.y_max - bound.y_min > 0.00001)
			offset = (center - bound.y_min) / (bound.y_max - bound.y_min);
		else return 0;
		break;
	case 2:
		center = (prim.z_max + prim.z_min) / 2;
		if (bound.z_max - bound.z_min > 0.00001)
			offset = (center - bound.z_min) / (bound.z_max - bound.z_min);
		else return 0;
		break;
	}
	int loc = nBucket * offset;
	if (loc == nBucket)
		loc = nBucket - 1;
	return loc;
}

inline double SAHHybrid::surfaceArea(BoundingBox& box) {
	double x = box.x_max - box.x_min;
	double y = box.y_max - box.y_min;
	double z = box.z_max - box.z_min;
	return (x * y + x * z + y * z) * 2;
}

int SAHHybrid::flattenBVHTree(HybridBVHNode* root, LinearBVHMemory& out, int* offset) {
	// std::cout << *offset << std::endl;
	if (root == nullptr) {
		return *offset;
	}
	LinearBVHNode* linearNode = &out[*offset];
	linearNode->bounds = root->boundingbox;
	int myOffset = (*offset)++;
	if (root->nObject > 0) {
		linearNode->primitivesOffset = root->firstObjOffset;
		linearNode->nPrimitives = root->nObject;
	}
	else {
		linearNode->nPrimitives = 0;
		flattenBVHTree(root->lchild, out, offset);
		linearNode->secondChildOffset = flattenBVHTree(root->rchild, out, offset);
	}
	return myOffset;
}

int SAHHybrid::countTreeNodes(HybridBVHNode* root) {
	if (root == nullptr) {
		return 0;
	}
	else {
		return countTreeNodes(root->lchild) + countTreeNodes(root->rchild) + 1;
	}
}

void SAHHybrid::BVHShowPrintLine(int depth, char c) {
	if (c == '-') {
		for (int i = 0; i <= depth; i++) {
			printf("--");
		}
		printf(">");
	}
	else if (c == ' ') {
		for (int i = 0; i <= depth; i++) {
			printf("  ");
		}
		printf(" ");
	}
}

void SAHHybrid::BVHShow(HybridBVHNode* BVHRoot, objectIndex indices, int maxdepth,
	int depth) {
	if (depth > maxdepth)
		return;
	if (BVHRoot == nullptr) {
		return;
	}
	BVHShowPrintLine(depth, '-');
	std::cout << "node depth: " << depth << std::endl;
	BVHShowPrintLine(depth, ' ');
	printf("bounding box: x_min = %.3f, x_max = %.3f,"
		"y_min = %.3f, y_max = %.3f,"
		"z_min = %.3f, z_max = %.3f\n",
		BVHRoot->boundingbox.x_min, BVHRoot->boundingbox.x_max,
		BVHRoot->boundingbox.y_min, BVHRoot->boundingbox.y_max,
		BVHRoot->boundingbox.z_min, BVHRoot->boundingbox.z_max);

	if (BVHRoot->nObject > 0) {
		BVHShowPrintLine(depth, ' ');
		std::cout << "type: " << "\033[33m" << "leaf" << "\033[37m" << std::endl;
		BVHShowPrintLine(depth, ' ');
		int offset = BVHRoot->firstObjOffset;
		BVHShowPrintLine(depth, ' ');
		std::cout << "offset begin: " << offset << " " << "length: " <<
			BVHRoot->nObject << std::endl;
		BVHShowPrintLine(depth, ' ');
		std::cout << "triangle index: ";
		for (int i = 0; i < BVHRoot->nObject; i++) {
			std::cout << indices[offset] << " ";
			offset++;
		}
		std::cout << std::endl;
	}
	else {
		BVHShowPrintLine(depth, ' ');
		std::cout << "type: " << "\033[34m" << "node" << "\033[37m" << std::endl;
		BVHShowPrintLine(depth, ' ');
	}
	BVHShow(BVHRoot->lchild, indices, maxdepth, depth + 1);
	BVHShow(BVHRoot->rchild, indices, maxdepth, depth + 1);
}

void SAHHybrid::BVHRemove(HybridBVHNode* root) {
	if (root != nullptr) {
		HybridBVHNode* left = root->lchild;
		HybridBVHNode* right = root->rchild;
		delete root;
		BVHRemove(left);
		BVHRemove(right);
	}
}
