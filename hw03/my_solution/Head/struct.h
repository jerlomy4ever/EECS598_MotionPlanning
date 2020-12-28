#include<iostream>
#include<vector>
#include <utility>  
#include <unordered_map>
#include <string> 
#include <algorithm>
#include "../alglib-3.16.0.cpp.gpl/cpp/src/stdafx.h"
#include "../alglib-3.16.0.cpp.gpl/cpp/src/alglibmisc.h"

using namespace std;
using namespace alglib;

class RRTNode
{
    public:
 vector<double> q;
 RRTNode* parent;
 //RRTNode* left;
 //RRTNode* right;
 RRTNode ();
 RRTNode (vector<double>);
 RRTNode (vector<double>,RRTNode*);
 vector<RRTNode*> back_track_except_self ();
 //int axis_intree;
 unsigned int index;

};


class NodeTree
{
    public:
 
 NodeTree ();

    void add(RRTNode*);

    void remove(unsigned int);

    vector<RRTNode*> get();

    RRTNode* get(unsigned int);

    vector<RRTNode*> back_track(unsigned int);

    kdtree kdTree();

    unsigned int size();

    private:
    unordered_map<unsigned int,RRTNode*> map_ip;
    //unordered_map<string,RRTNode*> map_sp;
    unsigned int index_count;

};

string tf_q2s(vector<double>);


pair<ae_int_t,real_2d_array> find_nearest_kd(kdtree,real_1d_array,int);

pair<ae_int_t,real_2d_array> find_nearest_kd_range(kdtree,real_1d_array,double);

RRTNode* get_kdTree(vector<RRTNode*>&,int);

RRTNode* find_nearest(RRTNode*,vector<double>);

vector<RRTNode*> find_nearest_range(RRTNode*,vector<double>, double );


