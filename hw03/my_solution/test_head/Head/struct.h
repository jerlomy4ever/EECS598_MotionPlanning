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
 RRTNode* left;
 RRTNode* right;
 RRTNode ();
 RRTNode (vector<double>);
 RRTNode (vector<double>,RRTNode*);
 vector<RRTNode*> back_track_except_self ();
 int axis_intree;
 double index;

};


class NodeTree
{
    public:
 
 NodeTree ();

    void add(RRTNode*);

    void remove(double);

    vector<RRTNode*> get();

    RRTNode* get(double);

    vector<RRTNode*> back_track(double);

    kdtree kdTree();

    double size();

    private:
    unordered_map<double,RRTNode*> map_ip;
    unordered_map<string,RRTNode*> map_sp;
    double index_count;

};

string tf_q2s(vector<double>);


pair<ae_int_t,real_2d_array> find_nearest_kd(kdtree,real_1d_array,int);

RRTNode* get_kdTree(vector<RRTNode*>&,int);

RRTNode* find_nearest(RRTNode*,vector<double>);

vector<RRTNode*> find_nearest_range(RRTNode*,vector<double>, double );


