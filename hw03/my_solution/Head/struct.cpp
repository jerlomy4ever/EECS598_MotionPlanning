#include "struct.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include <string> 
#include <algorithm>
#include <utility>  
#include "../alglib-3.16.0.cpp.gpl/cpp/src/stdafx.h"
#include "../alglib-3.16.0.cpp.gpl/cpp/src/alglibmisc.h"

using namespace alglib;
using namespace std;

RRTNode::RRTNode ()
{
    q={};
    parent=nullptr;
    index=0;
}

RRTNode::RRTNode (vector<double> input)
{
    q=input;
    parent=nullptr;
    index=0;
}

RRTNode::RRTNode (vector<double> input,RRTNode* p)
{
    q=input;
    parent=p;
    index=0;
}

vector<RRTNode*> RRTNode::back_track_except_self()
{
        vector<RRTNode*> ans;
        RRTNode* temp=parent;
        if(!temp){return ans;}
        ans.push_back(temp);
        while(temp->parent)
        {
            temp=temp->parent;
            ans.insert(ans.begin(),temp);
        }
        
        return ans;
}


string tf_q2s(vector<double> input)
{
    string ans;

    if(input.size()!=7)
    {
        cout<<"input vector dimension error";
        return ans;
    }

    ans=to_string(input[0]);

    for(int i=1;i<input.size();i++)
    {
        ans=ans+"|"+to_string(input[i]);
    }

    return ans;
}


NodeTree::NodeTree ()
{
    index_count=0;
}

NodeTree::~NodeTree ()
{
    index_count=0;
    for (auto& it : map_ip) {
        delete it.second;
    }
}

void NodeTree::add(RRTNode* input)
{
    input->index=index_count;
    map_ip[index_count]=input;
    index_count++;
}

void NodeTree::remove(unsigned int i)
{
    if(map_ip.count(i))
    {
    RRTNode* temp=map_ip[i];
    map_ip.erase(i);
    delete temp;
    }
    else{cout<<" NodeTree remove: not found in map_ip";}
}

vector<RRTNode*> NodeTree::get()
{

    vector<RRTNode*> nodes;
    if(map_ip.size()==0)
    {
        cout<<"Zero Node";
        return nodes;
        }

    for(auto& it:map_ip)
    {
        nodes.push_back(it.second);
    }
    return nodes;
}


RRTNode* NodeTree::get(unsigned int i)
{
    if(map_ip.count(i))
    {
    return map_ip[i];
    }
    else{cout<<" NodeTree get: not found in map_ip";
    return nullptr;}
}


vector<RRTNode*> NodeTree::back_track(unsigned int i)
{
    if(map_ip.count(i)==0)
    {cout<<" NodeTree back_track: not found in map_ip";
    return {};
    }
    else{
        vector<RRTNode*> ans;
        RRTNode* it=map_ip[i];
        ans.push_back(it);
        while(it->parent)
        {
            it=it->parent;
            ans.insert(ans.begin(),it);
        }
        
        return ans;
    }
}

unsigned int NodeTree::size()
{
    return map_ip.size();
}

kdtree NodeTree::kdTree()
{
    
    kdtree kdt;
    if(map_ip.size()==0)
    {
        cout<<"Zero Node";
        return kdt;
        }

    real_2d_array nodes;
    ae_int_t nx = 7;
    ae_int_t normtype = 2;
    ae_int_t ny = 1;
    nodes.setlength(map_ip.size(),8); //7 dof +index
    int i=0;
    for(auto& it:map_ip)
    {
        nodes[i][0]=it.second->q[0];
        nodes[i][1]=it.second->q[1];
        nodes[i][2]=it.second->q[2];
        nodes[i][3]=it.second->q[3];
        nodes[i][4]=it.second->q[4];
        nodes[i][5]=it.second->q[5];
        nodes[i][6]=it.second->q[6];
        nodes[i][7]=it.first;
        i++;
    }
    kdtreebuild(nodes, nx, ny, normtype, kdt);
    return kdt;
}


pair<ae_int_t,real_2d_array> find_nearest_kd(kdtree kdt,real_1d_array x,int num)
{
    ae_int_t k;
    real_2d_array r = "[[]]";
    k = kdtreequeryknn(kdt, x, num);
    kdtreequeryresultsxy(kdt, r);
    return make_pair(k,r);
}


pair<ae_int_t,real_2d_array> find_nearest_kd_range(kdtree kdt,real_1d_array x,double range)
{
    ae_int_t k;
    real_2d_array r = "[[]]";
    k = kdtreequeryrnn(kdt, x, range);
    kdtreequeryresultsxy(kdt, r);
    return make_pair(k,r);
}





