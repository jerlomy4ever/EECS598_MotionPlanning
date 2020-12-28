
#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <sstream>
#include <boost/format.hpp>
#include <openrave-core.h>
#include <stdio.h>     
#include <stdlib.h>    
#include <time.h>  
#include <boost/bind.hpp>
#include <iostream>
#include <string> 
#include <vector>
#include "Head/struct.h"
#include <typeinfo>
#include "alglib-3.16.0.cpp.gpl/cpp/src/stdafx.h"
#include "alglib-3.16.0.cpp.gpl/cpp/src/alglibmisc.h"
#include <math.h>  
#include <complex>   
#include <boost/lexical_cast.hpp>
#include <openrave/planningutils.h>
#include <openrave/plannerparameters.h>
#include <openrave/viewer.h>
#include <openrave/robot.h>
#include <chrono>
#include <algorithm> 
#include <time.h>   

using namespace OpenRAVE;
using namespace alglib;
using namespace std;
#define PI 3.14159265


class MyNewModule : public ModuleBase
{


public:
    MyNewModule(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("MyCommand",boost::bind(&MyNewModule::MyCommand,this,_1,_2),"This is an example command");
        RegisterCommand("setPose",boost::bind(&MyNewModule::setPose,this,_1,_2),"This is a setPose command");
        RegisterCommand("numbodies",boost::bind(&MyNewModule::NumBodies,this,_1,_2),"return number of bodies");
        RegisterCommand("load",boost::bind(&MyNewModule::Load, this,_1,_2),"loads a given file");
        RegisterCommand("rrt",boost::bind(&MyNewModule::rrt, this,_1,_2),"rrt connect success or fail");
        RegisterCommand("rrtextend",boost::bind(&MyNewModule::rrt_extend, this,_1,_2),"rrt extend success or fail");
        RegisterCommand("birrt",boost::bind(&MyNewModule::bi_rrt, this,_1,_2),"birrt success or fail");
        RegisterCommand("se",boost::bind(&MyNewModule::rrt_star_e, this,_1,_2),"rrt* (extend) success or fail");
        RegisterCommand("sc",boost::bind(&MyNewModule::rrt_star_c, this,_1,_2),"rrt* (connect) success or fail");
        
    }
    virtual ~MyNewModule() {}
    // new global parameters
    double max_extend;
    int max_steps;
    double ball_volume;
    double total_Lebesgue_measure;
    double gamma_selfdeine;
    vector<GraphHandlePtr> handles;	
    //
    bool rrt_star_c(std::ostream& sout, std::istream& sinput)
    {   
        unsigned long int size;
        std::string input_str;
        sinput >> input_str;
        vector<double> input_vec=decode_input(input_str);
        double bias=input_vec[1];
        double step_size=input_vec[2];
        vector<double> target_q(7);
        vector< RobotBasePtr> robots;
        GetEnv()->GetRobots(robots);
        RobotBasePtr robot=robots[0];
        vector<dReal> startconfig;
        robot->GetActiveDOFValues(startconfig);
        vector<double> start_q(7);
        int renew_scale=15;
        vector< dReal > lower;
        vector< dReal > upper;
        vector<double> low(7,0);
        vector<double> up(7,0);
        robot->GetActiveDOFLimits(lower,upper);
        for(int i=0;i<7;i++)
        {
            target_q[i]=double(input_vec[i+3]);
            start_q[i]=double(startconfig[i]);
            if(lower[i]<-1*PI)
            {
                lower[i]=-1*PI;
            }
            if(upper[i]>PI)
            {
                upper[i]=PI;
            }
            low[i]=double(lower[i]);
            up[i]=double(upper[i]);
        }
        NodeTree* nt=new NodeTree();
        RRTNode* nodeptr=new RRTNode(start_q);
        nt->add(nodeptr);
        kdtree kdt=nt->kdTree();
        size=nt->size();
        vector<double> cur_goal(7,0);
        real_1d_array cur_goal_real;
        cur_goal_real.setlength(7);
        pair<ae_int_t,real_2d_array> nearest_points; //nearest_points.first: number of points 
                                                     //nearest_points.second: points , each points is a row, [q, index]
        srand (time(NULL));
        vector<RRTNode*> extend;
        vector<RRTNode*> extend_renew;
        vector<RRTNode*> path;
        TrajectoryBasePtr traj=RaveCreateTrajectory (GetEnv(), "");
        traj->Init(robot->GetActiveConfigurationSpecification());
        unsigned int index_sample=0;
        max_steps=input_vec[0];
        max_extend= step_size * max_steps; //η
        ball_volume=pow(PI,3.5)/tgamma(4.5); //ζ d
        total_Lebesgue_measure=0;

        
        
        for(int i=0;i<7;i++){total_Lebesgue_measure=total_Lebesgue_measure+(up[i]-low[i]);}
        gamma_selfdeine=pow(2.0,7.0)*(1+1/7)*total_Lebesgue_measure;
        unsigned int nt_size;
        double neighbor_range;
        RRTNode* min_nodeptr;
        RRTNode* near_nodeptr;
        RRTNode* goal_nodeptr;
        unordered_map<unsigned int,long double> cost_map; // id->cost
        cost_map[nodeptr->index]=0;
        unsigned int goal_index=0;
        time_t timer;
        struct tm y2k = {0};
        double start;
        double end;
        double limit=900;
        y2k.tm_hour = 0;   y2k.tm_min = 0; y2k.tm_sec = 0;
        y2k.tm_year = 100; y2k.tm_mon = 0; y2k.tm_mday = 1;
        time(&timer);  
        start = difftime(timer,mktime(&y2k));
        bool reach=false;
        long double c; //c'
        int index_in_nn;
        int num;
        int connect_steps=8;

        //// get path
        while(true)
        {
            while (true)
            {
                if( ((double)rand() / RAND_MAX) >=bias/100)
                {
                    fRand_Halton(cur_goal,low, up,index_sample);
                    cur_goal[4]=0;
                    cur_goal[6]=0;
                    index_sample++;
                }
                else
                {
                    cur_goal=target_q;
                }
                for(int i=0;i<7;i++){cur_goal_real[i]=cur_goal[i];}
                nearest_points=find_nearest_kd(kdt,cur_goal_real,1);
                near_nodeptr=nt->get((nearest_points.second[0][7]));
                num=steer(robot,cur_goal,near_nodeptr,step_size);
                if(num>0){break;}
            }

            nodeptr=new RRTNode(cur_goal);
            nt_size=nt->size();
            neighbor_range=close_range(nt_size);
            min_nodeptr=near_nodeptr; //x_min
            nt->add(nodeptr);
            for(int i=0;i<7;i++){cur_goal_real[i]=nodeptr->q[i];}

            cost_map[nodeptr->index]=cost_map[min_nodeptr->index]+get_cost(min_nodeptr,nodeptr);

            if(neighbor_range!=0)
            {
                nearest_points=find_nearest_kd_range(kdt,cur_goal_real,neighbor_range);

                for(int i=1;i<=nearest_points.first;i++)
                {
                    near_nodeptr=nt->get((nearest_points.second[i-1][7])); //x_near
                    if(ObsFree(robot,near_nodeptr,nodeptr,step_size))
                    {
                        c=cost_map[near_nodeptr->index]+get_cost(near_nodeptr,nodeptr);
                        if(c<=cost_map[nodeptr->index])
                        {
                            cost_map[nodeptr->index]=c;
                            min_nodeptr=near_nodeptr;
                            index_in_nn=i;
                        }
                    }
                }
            }
            extend=connect(robot,min_nodeptr,nodeptr->q,step_size*connect_steps);
            if(extend.size()>1)
            {
                min_nodeptr=extend[0];
                for(int i=1;i<extend.size()-1;i++)
                {   
                    goal_nodeptr=min_nodeptr;
                    extend[i]->parent=min_nodeptr;
                    min_nodeptr=extend[i];
                    nt->add(min_nodeptr);
                    cost_map[min_nodeptr->index]=cost_map[goal_nodeptr->index]+get_cost(min_nodeptr,goal_nodeptr);
                }
                nodeptr->parent=min_nodeptr;

            }
            if(neighbor_range!=0)
            {
               
                for(int i=1;i<=nearest_points.first;i++)
                {
                    if(i==index_in_nn){continue;}
                    near_nodeptr=nt->get((nearest_points.second[i-1][7])); //x_near
                    if(ObsFree(robot,near_nodeptr,nodeptr,step_size) && cost_map[near_nodeptr->index]>cost_map[nodeptr->index]+get_cost(near_nodeptr,nodeptr))
                    {
                        extend=connect(robot,nodeptr,near_nodeptr->q,step_size*connect_steps);
                        if(extend.size()>1)
                        {
                            min_nodeptr=extend[0];
                            for(int i=1;i<extend.size()-1;i++)
                            {   
                                goal_nodeptr=min_nodeptr;
                                extend[i]->parent=min_nodeptr;
                                min_nodeptr=extend[i];
                                nt->add(min_nodeptr);
                                cost_map[min_nodeptr->index]=cost_map[goal_nodeptr->index]+get_cost(min_nodeptr,goal_nodeptr);
                            }
                            near_nodeptr->parent=min_nodeptr;
                        }
                    }
                }
            }

            if(!reach)
            {
                reach=close_or_not(nodeptr, target_q);
                if(reach)
                {
                    goal_index=nodeptr->index;
                }
            }

            time(&timer);  
            end = difftime(timer,mktime(&y2k));

            if(end-start>limit)
            {
                cout<<"Time counted by c++ : "; 
                cout<< end-start;
                cout<< "\n";

                if(reach)
                {
                    cout<<"RRT* (connect) : Success";
                    cout<< "\n";
                    sout<<1;
                    path=nt->back_track(goal_index);
                    break;
                }
                else
                {
                    cout<<"RRT* (connect) : Fail"; 
                    cout<< "\n";
                    sout<<0;
                    return false;
                }
            }

            if( (size*(renew_scale+100)/100) < nt->size()  )
            {
                size=nt->size();
                kdt=nt->kdTree();
            }
        }    
        
        /// draw
        RaveVector<float> color=RaveVector< float >(1, 0, 0, 1);
        draw_line(robot,path,color);
        /// execute path    
        for(int i=0;i<path.size();i++)
        {
            traj->Insert(i,path[i]->q);
        }
        planningutils::RetimeActiveDOFTrajectory(traj,robot);
        robot->GetController()->SetPath(traj);
        return true;
    }
    
    bool rrt_star_e(std::ostream& sout, std::istream& sinput)
    {   
        unsigned long int size;
        std::string input_str;
        sinput >> input_str;
        vector<double> input_vec=decode_input(input_str);
        double bias=input_vec[1];
        double step_size=input_vec[2];
        vector<double> target_q(7);
        vector< RobotBasePtr> robots;
        GetEnv()->GetRobots(robots);
        RobotBasePtr robot=robots[0];
        vector<dReal> startconfig;
        robot->GetActiveDOFValues(startconfig);
        vector<double> start_q(7);
        int renew_scale=15;
        vector< dReal > lower;
        vector< dReal > upper;
        vector<double> low(7,0);
        vector<double> up(7,0);
        robot->GetActiveDOFLimits(lower,upper);
        for(int i=0;i<7;i++)
        {
            target_q[i]=double(input_vec[i+3]);
            start_q[i]=double(startconfig[i]);
            if(lower[i]<-1*PI)
            {
                lower[i]=-1*PI;
            }
            if(upper[i]>PI)
            {
                upper[i]=PI;
            }
            low[i]=double(lower[i]);
            up[i]=double(upper[i]);
        }
        NodeTree* nt=new NodeTree();
        RRTNode* nodeptr=new RRTNode(start_q);
        nt->add(nodeptr);
        kdtree kdt=nt->kdTree();
        size=nt->size();
        vector<double> cur_goal(7,0);
        real_1d_array cur_goal_real;
        cur_goal_real.setlength(7);
        pair<ae_int_t,real_2d_array> nearest_points; //nearest_points.first: number of points 
                                                     //nearest_points.second: points , each points is a row, [q, index]
        srand (time(NULL));
        vector<RRTNode*> extend;
        vector<RRTNode*> extend_renew;
        vector<RRTNode*> path;
        TrajectoryBasePtr traj=RaveCreateTrajectory (GetEnv(), "");
        traj->Init(robot->GetActiveConfigurationSpecification());
        unsigned int index_sample=0;

        max_steps=input_vec[0];
        max_extend= step_size * max_steps; //η
        ball_volume=pow(PI,3.5)/tgamma(4.5); //ζ d
        total_Lebesgue_measure=0;
        
        for(int i=0;i<7;i++){total_Lebesgue_measure=total_Lebesgue_measure+(up[i]-low[i]);}
        gamma_selfdeine=pow(2.0,7.0)*(1+1/7)*total_Lebesgue_measure;
        unsigned int nt_size;
        double neighbor_range;
        RRTNode* min_nodeptr;
        RRTNode* near_nodeptr;
        RRTNode* goal_nodeptr;
        unordered_map<unsigned int,long double> cost_map; // id->cost
        cost_map[nodeptr->index]=0;
        unsigned int goal_index=0;
        time_t timer;
        struct tm y2k = {0};
        double start;
        double end;
        double limit=900;
        y2k.tm_hour = 0;   y2k.tm_min = 0; y2k.tm_sec = 0;
        y2k.tm_year = 100; y2k.tm_mon = 0; y2k.tm_mday = 1;
        time(&timer);  
        start = difftime(timer,mktime(&y2k));
        bool reach=false;
        long double c; //c'
        int index_in_nn;


        //// get path
        while(true)
        {
            while (true)
            {
                if( ((double)rand() / RAND_MAX) >=bias/100)
                {
                    fRand_Halton(cur_goal,low, up,index_sample);
                    cur_goal[4]=0;
                    cur_goal[6]=0;
                    index_sample++;
                }
                else
                {
                    cur_goal=target_q;
                }
                for(int i=0;i<7;i++){cur_goal_real[i]=cur_goal[i];}
                nearest_points=find_nearest_kd(kdt,cur_goal_real,1);
                near_nodeptr=nt->get((nearest_points.second[0][7]));
                int num=steer(robot,cur_goal,near_nodeptr,step_size);
                if(num>0){break;}
            }

            nodeptr=new RRTNode(cur_goal);
            nt_size=nt->size();
            neighbor_range=close_range(nt_size);
            
            min_nodeptr=near_nodeptr; //x_min
            nt->add(nodeptr);
            for(int i=0;i<7;i++){cur_goal_real[i]=nodeptr->q[i];}

            cost_map[nodeptr->index]=cost_map[min_nodeptr->index]+get_cost(min_nodeptr,nodeptr);

            if(neighbor_range!=0)
            {
                nearest_points=find_nearest_kd_range(kdt,cur_goal_real,neighbor_range);
                for(int i=1;i<=nearest_points.first;i++)
                {
                    near_nodeptr=nt->get((nearest_points.second[i-1][7])); //x_near
                    if(ObsFree(robot,near_nodeptr,nodeptr,step_size))
                    {
                        c=cost_map[near_nodeptr->index]+get_cost(near_nodeptr,nodeptr);
                        if(c<=cost_map[nodeptr->index])
                        {
                            cost_map[nodeptr->index]=c;
                            min_nodeptr=near_nodeptr;
                            index_in_nn=i;
                        }
                    }
                }
            }
            nodeptr->parent=min_nodeptr;

            if(neighbor_range!=0)
            {
                for(int i=1;i<=nearest_points.first;i++)
                {
                    if(i==index_in_nn){continue;}
                    near_nodeptr=nt->get((nearest_points.second[i-1][7])); //x_near
                    if(ObsFree(robot,near_nodeptr,nodeptr,step_size) && cost_map[near_nodeptr->index]>cost_map[nodeptr->index]+get_cost(near_nodeptr,nodeptr))
                    {
                        cost_map[near_nodeptr->index]=cost_map[nodeptr->index]+get_cost(near_nodeptr,nodeptr);
                        near_nodeptr->parent = nodeptr;
                    }
                }
            }

            if(!reach)
            {
                reach=close_or_not(nodeptr, target_q);
                if(reach)
                {
                    goal_index=nodeptr->index;
                }
            }

            time(&timer);  
            end = difftime(timer,mktime(&y2k));

            if(end-start>limit)
            {
                cout<<"Time counted by c++: "; 
                cout<< end-start;
                cout<< "\n";

                if(reach)
                {
                    cout<<"RRT* (extend) : Success";
                    cout<< "\n";
                    sout<<1;
                    path=nt->back_track(goal_index);
                    break;
                }
                else
                {
                    cout<<"RRT* (extend) : Fail"; 
                    cout<< "\n";
                    sout<<0;
                    return false;
                }
            }


            if( (size*(renew_scale+100)/100) < nt->size()  )
            {
                size=nt->size();
                kdt=nt->kdTree();
            }
        }    
        /// draw
        RaveVector<float> color=RaveVector< float >(1, 0, 0, 1);
        draw_line(robot,path,color);
        /// execute path    
        for(int i=0;i<path.size();i++)
        {
            traj->Insert(i,path[i]->q);
        }
        planningutils::RetimeActiveDOFTrajectory(traj,robot);
        robot->GetController()->SetPath(traj);
        return true;
    }
    
    long double get_cost(RRTNode* near_nodeptr,RRTNode* new_nodeptr)
    {
        long double length=0;
        long double temp;
        for(int i=0;i<7;i++)
        {
            temp=near_nodeptr->q[i]-new_nodeptr->q[i];
            length+= (temp* temp);
        }
        length=sqrt(length);
        return length;
    }
    
    double close_range(unsigned int nt_size)
    {
        double range;
        double size= double(nt_size);
        //range=min(max_extend, (double) pow( (gamma_selfdeine/ball_volume)*(log(size)/size) , double(1/7) ) );
        range=min(max_extend,0.3*pow((gamma_selfdeine/ball_volume)*(log(size)/size),1.0/7));
        
        return range;
    }

    bool ObsFree(RobotBasePtr robot,RRTNode* near_nodeptr,RRTNode* new_nodeptr,double& step_size)
    {
        vector<double> direction(7);
        long double length=0;
        for(int i=0;i<7;i++)
        {
            direction[i]=new_nodeptr->q[i]-near_nodeptr->q[i];
            length+=direction[i]*direction[i];
        }
        length=sqrt(length);
        for(int i=0;i<7;i++)
        {
            direction[i]=direction[i]/length;
        }
        int range=(ceil(length/step_size));
        vector<double> temp_goal_v(7);
        vector<dReal> temp_goal(7);
        int range_new=min(range,max_steps);
        for(int i=1;i<=range_new;i++)
        {
            if(i==range_new && range_new==range)
            {
                temp_goal_v=new_nodeptr->q;
                for(int j=0;j<7;j++)
                {
                    temp_goal[j]=temp_goal_v[j];
                }
            }
            else{
                for(int j=0;j<7;j++)
                {
                    temp_goal_v[j]=near_nodeptr->q[j]+i*step_size*direction[j];
                    temp_goal[j]=temp_goal_v[j];
                }
            }

            robot->SetActiveDOFValues(temp_goal);
            if (GetEnv()->CheckCollision(robot)==true || robot->CheckSelfCollision()==true)
            {
               return false;
            }

        }

        return true;
    }  
    
    int steer(RobotBasePtr robot,vector<double>& cur_goal,RRTNode* nodeptr,double& step_size)
    {
        vector<double> direction(7);
        long double length=0;
        for(int i=0;i<7;i++)
        {
            direction[i]=cur_goal[i]-nodeptr->q[i];
            length+=direction[i]*direction[i];
        }
        length=sqrt(length);
        for(int i=0;i<7;i++)
        {
            direction[i]=direction[i]/length;
        }
        int range=(ceil(length/step_size));
        vector<double> temp_goal_v(7);
        vector<dReal> temp_goal(7);
        int range_new=min(range,max_steps);
        if(range_new==0){return 0;}

        for(int i=1;i<=range_new;i++)
        {
            if(i==range_new && range_new==range)
            {

                temp_goal_v=cur_goal;
                for(int j=0;j<7;j++)
                {
                    temp_goal[j]=temp_goal_v[j];
                }
            }
            else{
                for(int j=0;j<7;j++)
                {
                    temp_goal_v[j]=nodeptr->q[j]+i*step_size*direction[j];
                    temp_goal[j]=temp_goal_v[j];
                }
            }
            robot->SetActiveDOFValues(temp_goal);
            if (GetEnv()->CheckCollision(robot)==true || robot->CheckSelfCollision()==true)
            {
                for(int j=0;j<7;j++)
                {
                    cur_goal[j]=temp_goal_v[j]-step_size*direction[j];
                }

               return i-1;
            }

        }

        cur_goal=temp_goal_v;
        return range_new;
    }
    
    bool bi_rrt(std::ostream& sout, std::istream& sinput)
    {   
        unsigned int size;
        unsigned int size2;
        std::string input_str;
        sinput >> input_str;
        vector<double> input_vec=decode_input(input_str);
        double bias=input_vec[1];
        double step_size=input_vec[2];
        vector<double> target_q(7);

        vector< RobotBasePtr> robots;
        GetEnv()->GetRobots(robots);
        RobotBasePtr robot=robots[0];
        vector<dReal> startconfig;
        robot->GetActiveDOFValues(startconfig);
        vector<double> start_q(7);

        int renew_scale=15;
        vector< dReal > lower;
        vector< dReal > upper;
        vector<double> low(7,0);
        vector<double> up(7,0);
        robot->GetActiveDOFLimits(lower,upper);
        for(int i=0;i<7;i++)
        {
            target_q[i]=double(input_vec[i+3]);
            start_q[i]=double(startconfig[i]);
            if(lower[i]<-1*PI)
            {
                lower[i]=-1*PI;
            }
            if(upper[i]>PI)
            {
                upper[i]=PI;
            }
            low[i]=double(lower[i]);
            up[i]=double(upper[i]);
        }

        
        NodeTree* nt=new NodeTree();
        RRTNode* nodeptr=new RRTNode(start_q);
        nt->add(nodeptr);
        kdtree kdt=nt->kdTree();
        size=nt->size();

        NodeTree* nt2=new NodeTree();
        RRTNode* nodeptr2=new RRTNode(target_q);
        nt2->add(nodeptr2);
        kdtree kdt2=nt2->kdTree();
        size2=nt2->size();

        vector<double> cur_goal(7,0);
        real_1d_array cur_goal_real;
        cur_goal_real.setlength(7);
        pair<ae_int_t,real_2d_array> nearest_points; //nearest_points.first: number of points 
                                                     //nearest_points.second: points , each points is a row, [q, index]
        srand (time(NULL));
        vector<RRTNode*> extend;
        vector<RRTNode*> path;
        vector<RRTNode*> path1;
        vector<RRTNode*> path2;
        TrajectoryBasePtr traj=RaveCreateTrajectory (GetEnv(), "");
        traj->Init(robot->GetActiveConfigurationSpecification());
        unsigned int index_sample=0;

        max_steps=input_vec[0];
        max_extend= step_size * max_steps;

        //// get path
        while(true)
        {
            while (true)
            {
                fRand_Halton(cur_goal,low, up,index_sample);
                index_sample++;
                
                for(int i=0;i<7;i++){cur_goal_real[i]=cur_goal[i];}
                nearest_points=find_nearest_kd(kdt,cur_goal_real,1);
                nodeptr=nt->get((nearest_points.second[0][7]));
                extend=connect(robot,nodeptr,cur_goal,step_size);
                if(extend.size()>1){break;}
            }
            nodeptr=extend[0];
            for(int i=1;i<extend.size();i++)
            {
                extend[i]->parent=nodeptr;
                nodeptr=extend[i];
                nt->add(nodeptr);
            }
            nodeptr=extend.back();


            for(int i=0;i<7;i++)
            {
                cur_goal_real[i]=nodeptr->q[i];
                cur_goal[i]=nodeptr->q[i];
            }
            nearest_points=find_nearest_kd(kdt2,cur_goal_real,1);
            nodeptr2=nt2->get((nearest_points.second[0][7]));
            extend=connect(robot,nodeptr2,cur_goal,step_size);
            if(extend.size()>1)
            {
                nodeptr2=extend[0];
                for(int i=1;i<extend.size();i++)
                {
                    extend[i]->parent=nodeptr2;
                    nodeptr2=extend[i];
                    nt2->add(nodeptr2);
                }
                nodeptr2=extend.back();
            }

            if( close_or_not(nodeptr, nodeptr2->q))
            {
                path1=nt->back_track(nodeptr->index);
                path2=nt2->back_track(nodeptr2->index);

                //combine
                if(close_or_not(path1[0], target_q))
                {
                    swap(path1,path2);
                }
                path=path1;
                for(int i=path2.size()-1;i>=0;i--)
                {
                    path.push_back(path2[i]);
                }

                break;
            }

            if( (size*(renew_scale+100)/100) < nt->size()  )
            {
                size=nt->size();
                kdt=nt->kdTree();
            }

            if( (size2*(renew_scale+100)/100) < nt2->size()  )
            {
                size2=nt2->size();
                kdt2=nt2->kdTree();
            }

            if(nt2->size()>nt->size())
            {
                swap(nt,nt2);
                swap(size,size2);
                swap(kdt,kdt2);
                swap(nodeptr,nodeptr2);
            }
        }    
        /// draw
        RaveVector<float> color=RaveVector< float >(1, 0, 0, 1);
        draw_line(robot,path,color);
        /// smooth
        int iterations=200;
        short_cut(robot,path,iterations,step_size);
        color=RaveVector< float >(0, 0, 1, 1);
        draw_line(robot,path,color);
        /// execute path    
        for(int i=0;i<path.size();i++)
        {
            traj->Insert(i,path[i]->q);
        }
        planningutils::RetimeActiveDOFTrajectory(traj,robot);
        robot->GetController()->SetPath(traj);
        sout<<"end";
        return true;
    }

    bool MyCommand(std::ostream& sout, std::istream& sinput)
    {
        std::string input;
        sinput >> input;
        sout << 123456;
        return true;
    }

    bool NumBodies(ostream& sout, istream& sinput)
    {
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        sout << vbodies.size();     // publish the results
        return true;
    }

    bool Load(ostream& sout, istream& sinput)
    {
        string filename;
        sinput >> filename;
        bool bSuccess = GetEnv()->Load(filename.c_str());     // load the file
        return bSuccess;
    }

    bool setPose(std::ostream& sout, std::istream& sinput)
    {   
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        vector< RobotBasePtr> robots;
        GetEnv()->GetRobots(robots);
        RobotBasePtr robot=robots[0];
        Transform tf=Transform(Vector(0.65839364,  0.68616871, -0.22320624, -0.21417118),Vector(0.23126595, -0.01218956,  0.1084143));
        robot->SetTransform	(tf);
        sout<<GetEnv()->CheckCollision(robot);
        return true;
    }

    // RRT Connect
    bool rrt(std::ostream& sout, std::istream& sinput)
    {   
        unsigned long int size;
        std::string input_str;
        sinput >> input_str;
        vector<double> input_vec=decode_input(input_str);
        double bias=input_vec[1];
        double step_size=input_vec[2];
        vector<double> target_q(7);
        vector< RobotBasePtr> robots;
        GetEnv()->GetRobots(robots);
        RobotBasePtr robot=robots[0];
        vector<dReal> startconfig;
        robot->GetActiveDOFValues(startconfig);
        vector<double> start_q(7);
        int renew_scale=15;
        vector< dReal > lower;
        vector< dReal > upper;
        vector<double> low(7,0);
        vector<double> up(7,0);
        robot->GetActiveDOFLimits(lower,upper);
        for(int i=0;i<7;i++)
        {
            target_q[i]=double(input_vec[i+3]);
            start_q[i]=double(startconfig[i]);
            if(lower[i]<-1*PI)
            {
                lower[i]=-1*PI;
            }
            if(upper[i]>PI)
            {
                upper[i]=PI;
            }
            low[i]=double(lower[i]);
            up[i]=double(upper[i]);
        }
        
        NodeTree* nt=new NodeTree();
        RRTNode* nodeptr=new RRTNode(start_q);
        nt->add(nodeptr);
        kdtree kdt=nt->kdTree();
        size=nt->size();

        vector<double> cur_goal(7,0);
        real_1d_array cur_goal_real;
        cur_goal_real.setlength(7);
        pair<ae_int_t,real_2d_array> nearest_points; //nearest_points.first: number of points 
                                                     //nearest_points.second: points , each points is a row, [q, index]
        srand (time(NULL));
        vector<RRTNode*> extend;
        vector<RRTNode*> path;

        TrajectoryBasePtr traj=RaveCreateTrajectory (GetEnv(), "");
        traj->Init(robot->GetActiveConfigurationSpecification());

        unsigned int index_sample=0;
        max_steps=input_vec[0];
        max_extend= step_size * max_steps;

        time_t timer;
        struct tm y2k = {0};
        double start;
        double end;
        double limit=200;
        y2k.tm_hour = 0;   y2k.tm_min = 0; y2k.tm_sec = 0;
        y2k.tm_year = 100; y2k.tm_mon = 0; y2k.tm_mday = 1;
        time(&timer);  
        start = difftime(timer,mktime(&y2k));
        bool reach=false;
        RRTNode* goal_nodeptr;


        //// get path
        while(true)
        {
            while (true)
            {
                if( ((double)rand() / RAND_MAX) >=bias/100)
                {
                    fRand_Halton(cur_goal,low, up,index_sample);
                    cur_goal[4]=0;
                    cur_goal[6]=0;
                    index_sample++;
                }
                else
                {
                    cur_goal=target_q;
                }
                for(int i=0;i<7;i++){cur_goal_real[i]=cur_goal[i];}
                nearest_points=find_nearest_kd(kdt,cur_goal_real,1);
                nodeptr=nt->get((nearest_points.second[0][7]));
                extend=connect(robot,nodeptr,cur_goal,step_size);
                if(extend.size()>1){break;}
            }
            nodeptr=extend[0];
            for(int i=1;i<extend.size();i++)
            {
                extend[i]->parent=nodeptr;
                nodeptr=extend[i];
                nt->add(nodeptr);
            }
            nodeptr=extend.back();

            if( close_or_not(nodeptr, target_q))
            {
                time(&timer);  
                end = difftime(timer,mktime(&y2k));
                cout<<"Time counted by c++ : "; 
                cout<< end-start;
                cout<< "\n";
                cout<<"RRT : Success"; 
                cout<< "\n";
                path=nt->back_track(nodeptr->index);
                break;
            }

            if( (size*(renew_scale+100)/100) < nt->size()  )
            {
                size=nt->size();
                kdt=nt->kdTree();
            }
        }    
        /// draw
        RaveVector<float> color=RaveVector< float >(1, 0, 0, 1);
        draw_line(robot,path,color);
        /// smooth
        int iterations=200;
        short_cut(robot,path,iterations,step_size);
        color=RaveVector< float >(0, 0, 1, 1);
        draw_line(robot,path,color);
        /// execute path    
        for(int i=0;i<path.size();i++)
        {
            traj->Insert(i,path[i]->q);
        }
        planningutils::RetimeActiveDOFTrajectory(traj,robot);
        robot->GetController()->SetPath(traj);
        sout<<"end";
        return true;
    }

    void short_cut(RobotBasePtr robot,vector<RRTNode*>& path,int iterations,double& step_size)
    {
        vector<RRTNode*> extend;
        for(int i=0;i<iterations;i++)
        {
            int s=path.size();
            int id1=rand() % s;
            int id2=rand() % s;
            while(id2==id1){id2=rand() % s;}
            if(id2<id1)
            {
                swap(id1,id2);
            }
            extend=connect(robot,path[id1],path[id2]->q,step_size);
            if( close_or_not(extend.back(), path[id2]->q))
            {
                    path.erase(path.begin()+id1,path.begin()+id2+1);
                    path.insert(path.begin()+id1,extend.begin(),extend.end());
            }
        }

        return; 
    }

    void draw_line(RobotBasePtr robot,vector<RRTNode*>& path,RaveVector<float>& color)
    {
        Transform tf;
        vector<RaveVector<float> > vpoints;
        for(int i=0;i<path.size();i++)
        {
            tf=GetEETransform(robot,path[i]);
            vpoints.push_back(tf.trans);
            handles.push_back(GetEnv()->drawlinestrip(&vpoints[0].x,vpoints.size(),sizeof(vpoints[0]),5,color));
        }

        return;
    }

    void fRand_Halton(vector<double>& cur_goal,vector<double>& fMin, vector<double>& fMax,unsigned int index)
    {
        vector<int> p={2,3,5,7,11,13,17};
        for(int i=0;i<7;i++)
        {
            cur_goal[i]=fMin[i] + Halton (index,p[i]) * (fMax[i] - fMin[i]);
        }

        return ;
    }

    double Halton (unsigned int index,int b)
    {
        double f=1;
        double r=0;

        while(index>0)
        {
            f=f/b;
            r=r+f*(index%b);
            index=floor(index/b);
        }
        return r;
    }

    Transform GetEETransform(RobotBasePtr robot,RRTNode* nodeptr)
    {
        vector<dReal> temp_goal(7);
        for(int j=0;j<7;j++)
            {
                temp_goal[j]=nodeptr->q[j];
            }
        robot->SetActiveDOFValues(temp_goal);
        RobotBase::ManipulatorPtr maniptr=robot->GetActiveManipulator();
        return maniptr->GetEndEffectorTransform ();
    }

    bool close_or_not(RRTNode* nodeptr, vector<double>& target_q)
    {
        double threshold=PI/(360);
        for(int i=0;i<7;i++)
        {
            if(abs(nodeptr->q[i]-target_q[i])>threshold)
            {
                return false;
            }
        }
        return true;
    }

    vector<RRTNode*> connect(RobotBasePtr robot,RRTNode* nodeptr,vector<double>& cur_goal,double step_size)
    {
        vector<RRTNode*> extend;
        extend.push_back(nodeptr);
        vector<double> direction(7);
        long double length=0;
        for(int i=0;i<7;i++)
        {
            direction[i]=cur_goal[i]-nodeptr->q[i];
            length+=direction[i]*direction[i];
        }
        length=sqrt(length);
        for(int i=0;i<7;i++)
        {
            direction[i]=direction[i]/length;
        }
        int range=(ceil(length/step_size));
        RRTNode* goal_ptr;
        vector<double> temp_goal_v(7);
        vector<dReal> temp_goal(7);
        int range_new=min(range,max_steps);
        for(int i=1;i<=range_new;i++)
        {
            if(i==range_new && range_new==range)
            {
                temp_goal_v=cur_goal;
                for(int j=0;j<7;j++)
                {
                    temp_goal[j]=temp_goal_v[j];
                }
            }
            else{
                for(int j=0;j<7;j++)
                {
                    temp_goal_v[j]=nodeptr->q[j]+i*step_size*direction[j];
                    temp_goal[j]=temp_goal_v[j];
                }
            }
            robot->SetActiveDOFValues(temp_goal);
            if (GetEnv()->CheckCollision(robot)==true || robot->CheckSelfCollision()==true)
            {
               return extend;
            }
            goal_ptr=new RRTNode(temp_goal_v);
            extend.push_back(goal_ptr);

        }
        return extend;
    }
    
    void fRand(vector<double>& cur_goal,vector<double>& fMin, vector<double>& fMax)
    {
        for(int i=0;i<7;i++)
        {
            double f = (double) rand() / RAND_MAX;
            cur_goal[i]=fMin[i] + f * (fMax[i] - fMin[i]);
        }
        cur_goal[4]=0;
        cur_goal[6]=0;
        return ;
    }

    vector<double> decode_input(string input)
    {
        vector<double> ans;
        string temp;
        int index=0;
        while(index<input.length())
        {
            if(input[index]=='|')
            {
                index++;
                ans.push_back(stod(temp));
                temp="";
                continue;
            }
            temp+=input[index];
            index++;
        }
        ans.push_back(stod(temp));
        return ans;
    }

    // RRT Extend
    bool rrt_extend(std::ostream& sout, std::istream& sinput)
    {   
        unsigned long int size;
        std::string input_str;
        sinput >> input_str;
        vector<double> input_vec=decode_input(input_str);
        double bias=input_vec[1];
        double step_size=input_vec[2];
        vector<double> target_q(7);
        vector< RobotBasePtr> robots;
        GetEnv()->GetRobots(robots);
        RobotBasePtr robot=robots[0];
        vector<dReal> startconfig;
        robot->GetActiveDOFValues(startconfig);
        vector<double> start_q(7);
        int renew_scale=15;
        vector< dReal > lower;
        vector< dReal > upper;
        vector<double> low(7,0);
        vector<double> up(7,0);
        robot->GetActiveDOFLimits(lower,upper);
        for(int i=0;i<7;i++)
        {
            target_q[i]=double(input_vec[i+3]);
            start_q[i]=double(startconfig[i]);
            if(lower[i]<-1*PI)
            {
                lower[i]=-1*PI;
            }
            if(upper[i]>PI)
            {
                upper[i]=PI;
            }
            low[i]=double(lower[i]);
            up[i]=double(upper[i]);
        }
        
        NodeTree* nt=new NodeTree();
        RRTNode* nodeptr=new RRTNode(start_q);
        nt->add(nodeptr);
        kdtree kdt=nt->kdTree();
        size=nt->size();
        vector<double> cur_goal(7,0);
        real_1d_array cur_goal_real;
        cur_goal_real.setlength(7);
        pair<ae_int_t,real_2d_array> nearest_points; //nearest_points.first: number of points 
                                                     //nearest_points.second: points , each points is a row, [q, index]
        srand (time(NULL));
        vector<RRTNode*> extend;
        vector<RRTNode*> extend_renew;
        vector<RRTNode*> path;
        TrajectoryBasePtr traj=RaveCreateTrajectory (GetEnv(), "");
        traj->Init(robot->GetActiveConfigurationSpecification());
        unsigned int index_sample=0;
        
        max_steps=input_vec[0];
        max_extend= step_size * max_steps; // η
        ball_volume=pow(PI,3.5)/tgamma(4.5); // ζ d
        total_Lebesgue_measure=0;
        for(int i=0;i<7;i++){total_Lebesgue_measure=total_Lebesgue_measure+(up[i]-low[i]);}
        gamma_selfdeine=pow(2,7)*(1+1/7)*total_Lebesgue_measure;
        unsigned int nt_size;
        double neighbor_range;
        RRTNode* min_nodeptr;
        RRTNode* near_nodeptr;
        RRTNode* goal_nodeptr;
        unordered_map<unsigned int,long double> cost_map; // id->cost
        cost_map[nodeptr->index]=0;
        unsigned int goal_index=0;
        time_t timer;
        struct tm y2k = {0};
        double start;
        double end;
        double limit=900;
        y2k.tm_hour = 0;   y2k.tm_min = 0; y2k.tm_sec = 0;
        y2k.tm_year = 100; y2k.tm_mon = 0; y2k.tm_mday = 1;
        time(&timer);  
        start = difftime(timer,mktime(&y2k));
        bool reach=false;
        long double c; //c'
        //// get path
        while(true)
        {
            while (true)
            {
                if( ((double)rand() / RAND_MAX) >=bias/100)
                {
                    fRand_Halton(cur_goal,low, up,index_sample);
                    cur_goal[4]=0;
                    cur_goal[6]=0;
                    index_sample++;
                }
                else
                {
                    cur_goal=target_q;
                }
                for(int i=0;i<7;i++){cur_goal_real[i]=cur_goal[i];}
                nearest_points=find_nearest_kd(kdt,cur_goal_real,1);
                near_nodeptr=nt->get((nearest_points.second[0][7]));
                int num=steer(robot,cur_goal,near_nodeptr,step_size);
                if(num>0){break;}
            }
            nodeptr=new RRTNode(cur_goal);
            nt->add(nodeptr);
            nodeptr->parent=near_nodeptr;
            cout<<"size : "; 
            cout<< nt->size();
            cout<< "\n";
            if(!reach)
            {
                reach=close_or_not(nodeptr, target_q);
                if(reach)
                {
                    goal_index=nodeptr->index;
                }
            }
            if(reach)
                {
                    time(&timer);  
                    end = difftime(timer,mktime(&y2k));
                    cout<<"Time counted by c++ : "; 
                    cout<< end-start;
                    cout<< "\n";
                    cout<<"RRT Extend : Success"; 
                    cout<< "\n";
                    path=nt->back_track(nodeptr->index);
                    break;
                }
            if( (size*(renew_scale+100)/100) < nt->size()  )
            {
                size=nt->size();
                kdt=nt->kdTree();
            }
        }    
        /// draw
        RaveVector<float> color=RaveVector< float >(1, 0, 0, 1);
        draw_line(robot,path,color);
        // shortcut
        int iterations=200;
        short_cut(robot,path,iterations,step_size);
        color=RaveVector< float >(0, 0, 1, 1);
        draw_line(robot,path,color);
        /// execute path    
        for(int i=0;i<path.size();i++)
        {
            traj->Insert(i,path[i]->q);
        }
        planningutils::RetimeActiveDOFTrajectory(traj,robot);
        robot->GetController()->SetPath(traj);
        return true;
    }
};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "mynewmodule" ) {
        return InterfaceBasePtr(new MyNewModule(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("MyNewModule");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

