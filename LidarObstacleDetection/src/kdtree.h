#include "render/render.h"


// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	typename PointT point;
	int id;
	typename Node<PointT>* left;
	typename Node<PointT>* right;

	Node(typename PointT arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{
	}

	~Node()
	{
		delete left;
		delete right;
	}
};

template<typename PointT>
struct KdTree
{
	typename Node<PointT>* root;

	KdTree()
		: root(NULL)
	{
	}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(typename Node<PointT>** node, typename PointT point, int id, uint16_t depth) {
		if (*node == NULL) {
			*node = new Node<PointT>(point, id);
			return;
		}

		uint16_t dim = depth % 3; // get current dimension (x or y)
		if (point.data[dim] < ((*node)->point.data[dim])) // left if lesser in the dimension
		{
			insertHelper(&((*node)->left), point, id, depth + 1);
		}
		else // right if higher in the dimension 
		{
			insertHelper(&((*node)->right), point, id, depth + 1);
		}
	}

	void insertCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		for (int idx = 0; idx < cloud->points.size(); idx++)
			insertHelper(&root, cloud->points[idx], idx, 0);

	}

	void searchHelper(typename PointT target, std::vector<int>& ids, typename Node<PointT>* node, float distanceTol, uint16_t depth) {
		if (node != NULL) {

			float x{ node->point.x }, y{ node->point.y }, z{ node->point.z };
			if (((x >= (target.x - distanceTol))
				&& (x <= (target.x + distanceTol)))
				&& ((y >= (target.y - distanceTol))
					&& (y <= (target.y + distanceTol)))
				&& ((z >= (target.z - distanceTol))
					&& (z <= (target.z + distanceTol)))
				)
			{
				// inside the 2 x distanceTol box around target
				float dist{ sqrt(((x - target.x) * (x - target.x)) + ((y - target.y) * (y - target.y)) + ((z - target.z) * (z - target.z))) };
				if (dist <= distanceTol) ids.push_back(node->id);
			}
			// if current dim < node --> go left
			if ((target.data[depth % 3] - distanceTol) < node->point.data[depth % 3]) {
				searchHelper(target, ids, node->left, distanceTol, depth + 1);
			}
			// if current dim > node --> go right
			if ((target.data[depth % 3] + distanceTol) > node->point.data[depth % 3]) {
				searchHelper(target, ids, node->right, distanceTol, depth + 1);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(typename PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, ids, root, distanceTol, 0);
		return ids;
	}


};




#pragma once
