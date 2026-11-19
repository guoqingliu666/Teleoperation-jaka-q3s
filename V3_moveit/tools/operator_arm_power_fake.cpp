// Offline substitute for the vendor C API. Never links the real SDK.
#include "v3_moveit/jaka_sdk.hpp"
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
static std::string scenario(){const char* v=std::getenv("V3_ARM_CASE");return v?v:"";}
static bool powered=false,enabled=false;static int power_calls=0,enable_calls=0,tool_calls=0,current_tool=1;
static void numbers(const char* key,double* values,int count){std::istringstream s(std::getenv(key));for(int i=0;i<count;i++)s>>values[i];}
extern "C" {
errno_t create_handler(const char*,JKHD* h){*h=1;powered=scenario()!="off";enabled=scenario()=="already_enabled";current_tool=scenario()=="tool"?3:1;return scenario()=="connect_error"?-3:scenario()=="edg_owned"?-58:0;}
errno_t destory_handler(const JKHD*){std::cout<<"{\"event\":\"fake_calls\",\"power_on\":"<<power_calls<<",\"enable\":"<<enable_calls<<",\"set_tool_id\":"<<tool_calls<<"}\n";return 0;}
errno_t get_robot_status_simple(const JKHD*,RobotStatus_simple* s){*s={};s->powered_on=powered;s->enabled=enabled;if(scenario()=="alarm")s->errcode=9;return 0;}
errno_t get_motion_status(const JKHD*,MotionStatus* s){*s={};s->inpos=1;s->isInEstop=scenario()=="estop";s->isInCollision=scenario()=="collision";s->queue=scenario()=="queue";return 0;}
errno_t is_in_servomove(const JKHD*,BOOL* v){*v=scenario()=="servo";return 0;}
errno_t is_in_drag_mode(const JKHD*,BOOL* v){*v=scenario()=="drag";return 0;}
errno_t get_program_state(const JKHD*,ProgramState* v){*v=scenario()=="program"?PROGRAM_RUNNING:PROGRAM_IDLE;return 0;}
errno_t get_actual_joint_position(const JKHD*,JointValue* q){*q={};return scenario()=="feedback_error"?5:0;}
errno_t get_tool_id(const JKHD*,int* v){*v=current_tool;return 0;}
errno_t set_tool_id(const JKHD*,int v){tool_calls++;if(scenario()=="tool_error")return 8;if(scenario()!="tool_unconfirmed")current_tool=v;return 0;}
errno_t get_tool_data(const JKHD*,int id,CartesianPose* p){*p={};p->tran.z=scenario()=="tcp"?100:200;p->tran.x=id==3?12.345678901234567:0;return scenario()=="missing_tool"&&id==3?9:0;}
errno_t get_payload(const JKHD*,PayLoad* p){double v[4];numbers("V3_ARM_PAYLOAD",v,4);*p={};p->mass=v[0]+(scenario()=="payload"?1:0);p->centroid={v[1]+(scenario()=="centroid"?1:0),v[2],v[3]};return 0;}
errno_t get_collision_level(const JKHD*,int* v){*v=scenario()=="collision_disabled"?0:3;return 0;}
errno_t kine_forward(const JKHD*,const JointValue*,CartesianPose* p){double v[6];numbers("V3_ARM_FK",v,6);p->tran={v[0]+(scenario()=="model"?10:0),v[1],v[2]};p->rpy={v[3],v[4],v[5]};return 0;}
errno_t power_on(const JKHD*){power_calls++;powered=true;return scenario()=="command_error"?7:0;}
errno_t enable_robot(const JKHD*){enable_calls++;enabled=true;return scenario()=="command_error"?7:0;}
}
