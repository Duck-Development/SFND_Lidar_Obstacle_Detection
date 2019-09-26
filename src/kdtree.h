/* \author Aaron Brown */
#include <pcl/point_types.h>
// Structure to represent node of kd tree
template <typename PointT>
struct Node
{
    PointT point;
	int id;
	Node* left;
	Node* right;


    Node<PointT>(PointT pnt, int setId)
    :	point(pnt) , id(setId), left(NULL), right(NULL)
    { }
};
template <typename PointT>
void insertHelper(Node<PointT> *&parent , Node<PointT> *&child , int depth = 0 )
{
    if (parent == nullptr)
    {
        parent = child;
        return;
    }
    auto xyz =  depth % 3;
    depth++;

        if(parent->point.data[xyz] < child->point.data[xyz] ) // left
        {

           insertHelper( parent->right, child , depth );
        }
        else
        {
            insertHelper(parent->left , child , depth );
        }
    }

template<typename PointT>
void searchHelper(Node<PointT> *&node , const PointT &target, float distanceTol ,std::vector<int> &ids , int depth )
{
    if (node == nullptr){ // leaf reached
        return;
    }
    auto xyz =  depth % 3;
    depth++;
    auto nodePos = node->point;
    if((target.data[xyz] - nodePos.data[xyz] ) > distanceTol ) // Too fare left
    {
        searchHelper(node->right , target, distanceTol ,ids , depth );
    } else if (nodePos.data[xyz] - (target.data[xyz] ) > distanceTol ) // Too fare right
        searchHelper(node->left , target, distanceTol ,ids , depth );
    else {

        // inside of dimmension
        auto next1 = (xyz+1)%3;
        auto next2 = (xyz+2)%3;
        if ((abs(nodePos.data[next1] - (target.data[next1] )) <= distanceTol) && (abs(nodePos.data[next2] - (target.data[next2] )) <= distanceTol)  )
        {
            if(sqrt(pow(nodePos.x -target.x , 2  ) + pow(nodePos.y -target.y , 2  )  + pow(nodePos.z -target.z , 2  )  ) <=distanceTol  )
            {
                ids.push_back(node->id);
            }
        }
        searchHelper(node->right , target, distanceTol ,ids , depth );
        searchHelper(node->left , target, distanceTol ,ids , depth );
    }
}


template <typename PointT>
struct KdTree
{
    Node<PointT>* root;

	KdTree()
    : root(nullptr)
	{}


    void insert(PointT point, int id)
    {
		// TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        auto child = new Node<PointT>(point , id);
        insertHelper(root , child );

	}

	// return a list of point ids in the tree that are within distance of target

    std::vector<int> search( PointT target, float distanceTol)
	{
		std::vector<int> ids;
        searchHelper(root , target, distanceTol ,ids , 0 );

		return ids;
	}
	

};




