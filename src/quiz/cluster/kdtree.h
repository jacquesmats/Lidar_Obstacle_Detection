/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	
  	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
    {
      if(*node==NULL)
        *node = new Node(point,id);
      else{
        uint cd = depth % 2;
        
        if(point[cd] < ((*node)->point[cd]))
          insertHelper(&((*node)->left), depth+1, point, id);
        else
          insertHelper(&((*node)->right), depth+1, point, id);
      }
    }
    
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root,0,point,id);
	}
	
  	void searchHelper(Node* node, std::vector<float> target, float distanceTol, uint depth, std::vector<int>& ids)
    {
      if(node!=NULL){
		  
          float box[4];

          box[0] = target[0] - distanceTol;
          box[1] = target[0] + distanceTol;

          box[2] = target[1] + distanceTol;
          box[3] = target[1] - distanceTol;
        
          float x = node->point[0];
          float y = node->point[1];

          if((x >= box[0] && x <= box[1]) &&
            (y <= box[2] && y >= box[3])){
            // Why do we need distance here?
            float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) +
                                  (node->point[1] - target[1]) * (node->point[1] - target[1]));
            if (distance <= distanceTol)
            {
                ids.push_back(node->id);
            }
          }

          // Look right and discard left or look left and discard right
          if ((target[depth % 2] - distanceTol) < node->point[depth % 2])
          {
              searchHelper(node->left, target, distanceTol, depth+1, ids);
          }
          if ((target[depth % 2] + distanceTol) > node->point[depth % 2])
          {
              searchHelper(node->right, target, distanceTol, depth+1, ids);
          }

      }    
    }
  
  
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
      
      	searchHelper(root, target, distanceTol, 0, ids);
            
		return ids;
	}


};




