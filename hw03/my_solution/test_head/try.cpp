#include<iostream>
#include"Head/struct.h"
#include <typeinfo>

#include "alglib-3.16.0.cpp.gpl/cpp/src/stdafx.h"
#include "alglib-3.16.0.cpp.gpl/cpp/src/alglibmisc.h"

using namespace std;
using namespace alglib;

int main(){
 vector<double> q={0,1,2,3,4,5,6};
 RRTNode* nodeptr=new RRTNode(q);
 NodeTree* treeptr=new NodeTree();
 treeptr->add(nodeptr);

 q={10,11,12,13,14,15,16};
 nodeptr=new RRTNode(q);
 treeptr->add(nodeptr);


 q={110,111,112,113,114,115,116};
 nodeptr=new RRTNode(q);
 treeptr->add(nodeptr);
    cout<<treeptr->size();
    cout<<"\n";
    kdtree kdt=treeptr->kdTree();
  real_1d_array target;
  target="[10,10,10,10,10,10,10]";
    pair<ae_int_t,real_2d_array> ans=find_nearest_kd(kdt,target,1);

  cout<<int(ans.first);
  cout<<"\n";
  for(int i=0;i<ans.second.rows();i++)
      {
        for(int j=0;j<ans.second.cols();j++)
      {
        cout<<(ans.second[i][j]);
        cout<<"\n";
        cout << typeid(ans.second[i][j]).name() << endl;
      }
      }



  return 0 ; 
}
