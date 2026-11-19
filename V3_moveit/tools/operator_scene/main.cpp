// Display-only RViz renderer. ROS domain 91; never publishes a control command.
#include <QApplication>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QTimer>
#include <QVBoxLayout>
#include <QLabel>
#include <QDockWidget>
#include <QToolBar>
#include <QMenuBar>
#include <QStatusBar>
#include <chrono>
#include <cmath>
#include <rviz_common/visualization_frame.hpp>
#include <QDateTime>
#include <OgreViewport.h>
#include <OgreCamera.h>
#include <OgreRenderTarget.h>
#include <OgreColourValue.h>
#include <OgreTextureManager.h>
#include <OgreHardwarePixelBuffer.h>
#include <OgreRenderTexture.h>
#include <QMouseEvent>
#include <QWheelEvent>
#include <rviz_common/viewport_mouse_event.hpp>
#include <sys/mman.h>
#include <fcntl.h>
#include <cstring>
#include <vector>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <robot_state_publisher/robot_state_publisher.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_rendering/render_window.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "target_markers.hpp"
#include "target_overlay.hpp"
#include <rviz_common/frame_manager_iface.hpp>
class DisplayState : public robot_state_publisher::RobotStatePublisher {
public:
 explicit DisplayState(const rclcpp::NodeOptions &o):RobotStatePublisher(o){}
 void pose(const std::map<std::string,double>& q){publishTransforms(q,now());}
};
int main(int argc,char**argv){
 if(argc!=2)return 2;
 const QString root=argv[1];
 // This process cannot share the robot's control domain.
 setenv("ROS_DOMAIN_ID","91",1);setenv("ROS_LOCALHOST_ONLY","1",1);
 rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
 QApplication app(argc,argv);
 auto abstraction=std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("v3_operator_scene");
 QFile model(root+"/.cache/operator/display.urdf");if(!model.open(QIODevice::ReadOnly))return 3;
 auto state=std::make_shared<DisplayState>(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("robot_description",model.readAll().toStdString())}));
 rviz_common::VisualizationFrame window(abstraction);window.setApp(&app);window.setSplashPath("");
 window.initialize(abstraction,root+"/tools/operator_scene/scene.rviz");window.setWindowTitle("V3 Offscreen Renderer");window.resize(880,410);
 for(auto dock:window.findChildren<QDockWidget*>())dock->hide();
 for(auto toolbar:window.findChildren<QToolBar*>())toolbar->hide();
 window.menuBar()->hide();window.statusBar()->hide();window.setHideButtonVisibility(false);
 auto manager=window.getManager();auto render=window.getRenderWindow();manager->setFixedFrame("world");
 auto grid=manager->createDisplay("rviz_default_plugins/Grid","Ground grid",true);
 grid->subProp("Reference Frame")->setValue("base_link");grid->subProp("Cell Size")->setValue(.1);grid->subProp("Plane Cell Count")->setValue(30);grid->subProp("Color")->setValue(QColor(71,83,101));
 auto robot=manager->createDisplay("rviz_default_plugins/RobotModel","Live robot",true);
 robot->subProp("Description Source")->setValue("Topic");robot->subProp("Description Topic")->setValue("/robot_description");
 robot->subProp("Collision Enabled")->setValue(false);robot->subProp("Visual Enabled")->setValue(true);
 auto force_pub=state->create_publisher<visualization_msgs::msg::MarkerArray>("/v3/display_hand_forces",1);
 auto force_display=manager->createDisplay("rviz_default_plugins/MarkerArray","Hand force (local illustration)",true);
 force_display->subProp("Topic")->setValue("/v3/display_hand_forces");
 auto target_pub=state->create_publisher<visualization_msgs::msg::MarkerArray>("/v3/display_hand_target",1);
 auto target_layer=manager->createDisplay("rviz_default_plugins/MarkerArray","Hand target / 5 second trail",true);
 target_layer->subProp("Topic")->setValue("/v3/display_hand_target");
 auto target_axes=std::make_unique<TargetOverlay>(manager->getSceneManager());
 auto base_axes=manager->createDisplay("rviz_default_plugins/Axes","Robot base coordinates",true);
 base_axes->subProp("Reference Frame")->setValue("base_link");base_axes->subProp("Length")->setValue(.15);base_axes->subProp("Radius")->setValue(.002);
 auto axes=manager->createDisplay("rviz_default_plugins/Axes","TCP",true);axes->subProp("Reference Frame")->setValue("tcp");axes->subProp("Length")->setValue(.08);axes->subProp("Radius")->setValue(.002);
 manager->getToolManager()->setCurrentTool(manager->getToolManager()->addTool("rviz_default_plugins/MoveCamera"));
 manager->getViewManager()->setCurrentViewControllerType("rviz_default_plugins/Orbit");
 auto view=manager->getViewManager()->getCurrent();view->subProp("Distance")->setValue(1.25);view->subProp("Yaw")->setValue(.8);view->subProp("Pitch")->setValue(.35);
 view->subProp("Focal Point")->subProp("X")->setValue(.1);view->subProp("Focal Point")->subProp("Y")->setValue(.25);view->subProp("Focal Point")->subProp("Z")->setValue(.98);
 QTimer::singleShot(800,[view](){
   view->subProp("Distance")->setValue(1.25);view->subProp("Yaw")->setValue(.8);view->subProp("Pitch")->setValue(.35);
   view->subProp("Focal Point")->subProp("X")->setValue(.1);view->subProp("Focal Point")->subProp("Y")->setValue(.25);view->subProp("Focal Point")->subProp("Z")->setValue(.98);
 });
 Ogre::ColourValue background(.075,.10,.15);rviz_rendering::RenderWindowOgreAdapter::setBackgroundColor(render, &background);
 // Render to a GPU texture, then deliver pixels to the actual Qt canvas.
 // Never map a separate top-level window; QWidget overlays remain compositable.
 manager->startUpdate();window.hide();
 constexpr size_t capacity=64+1600*900*4;
 int fd=::open((root+"/.cache/operator/scene_pixels.bin").toStdString().c_str(),O_RDWR|O_CREAT,0600);
 if(fd<0 || ftruncate(fd,capacity)!=0)return 4;
 auto memory=static_cast<unsigned char*>(mmap(nullptr,capacity,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0));if(memory==MAP_FAILED)return 5;
 struct Header{uint64_t seq;uint32_t pid,width,height,stride;double updated,distance;};
 auto header=reinterpret_cast<Header*>(memory);std::memset(memory,0,64);header->pid=getpid();
 Ogre::TexturePtr texture;Ogre::RenderTexture* target=nullptr;int width=0,height=0;uint64_t sequence=0;double input_seq=0;
 std::vector<unsigned char> pixels;

 QTimer timer;pid_t parent=getppid();std::map<std::string,double> q;double target_session_floor=0;
 // Neutral display pose only. The parent labels this as preview until fresh feedback.
 QFile metadata(root+"/src/v3_moveit/config/model_metadata.json");metadata.open(QIODevice::ReadOnly);auto initial=QJsonDocument::fromJson(metadata.readAll()).object()["q_reference_rad"].toArray();for(int i=0;i<6;i++)q["joint_"+std::to_string(i+1)]=initial.size()==6?initial[i].toDouble():0.;
 QFile handmodel(root+"/.cache/operator/display.urdf");
 for(const char* name:{"finger11","finger12","finger13","finger14","finger21","finger22","finger23","finger31","finger32","finger33","finger41","finger42","finger43","finger51","finger52","finger53"})q[std::string("DH_")+name]=0.;
 QObject::connect(&timer,&QTimer::timeout,[&](){
   if(getppid()!=parent){app.quit();return;}
   QFile f(root+"/.cache/operator/scene_state.json");if(f.open(QIODevice::ReadOnly)){
     auto d=QJsonDocument::fromJson(f.readAll()).object();auto a=d["joint_radians"].toArray();
     target_session_floor=d["target_session_started"].toDouble();
     double now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
     if(d["fresh"].toBool() && now-d["updated"].toDouble()<.5 && a.size()==6)for(int i=0;i<6;i++)q["joint_"+std::to_string(i+1)]=a[i].toDouble();
     if(d["reset_view"].toDouble()>0 && d["reset_view"].toDouble()>window.property("reset_view").toDouble()){
       window.setProperty("reset_view",d["reset_view"].toDouble());view->subProp("Distance")->setValue(1.25);view->subProp("Yaw")->setValue(.8);view->subProp("Pitch")->setValue(.35);view->subProp("Focal Point")->subProp("X")->setValue(.1);view->subProp("Focal Point")->subProp("Y")->setValue(.25);view->subProp("Focal Point")->subProp("Z")->setValue(.98);
     }
   }
   QFile handfile(root+"/.cache/operator/media/status.json");
   if(handfile.open(QIODevice::ReadOnly)){
     auto media=QJsonDocument::fromJson(handfile.readAll()).object();auto d=media["devices"].toObject()["hand"].toObject();
     double now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
     if(d["ok"].toBool() && now-media["updated"].toDouble()<1.5 && now-d["updated"].toDouble()<1.5){
       auto names=d["joint_names"].toArray();auto angles=d["position"].toArray();
       for(int i=0;i<names.size() && i<angles.size();i++)q["DH_"+names[i].toString().remove("DH_").toStdString()]=angles[i].toDouble();
     }
   }
   // The owner's fresh raw joint feedback takes priority over external ROS monitoring.
   double hand_now=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
   QJsonObject grasp_data;
   for(const auto& file:{"/.cache/operator/dh116_status.json","/.cache/operator/grasp_state.json"}){
     QFile input(root+file);if(!input.open(QIODevice::ReadOnly))continue;
     auto d=QJsonDocument::fromJson(input.readAll()).object();double age=hand_now-d["updated"].toDouble();
     if(d["ok"].toBool() && age>=0 && age<.08){
       auto names=d["joint_names"].toArray();auto angles=d["position"].toArray();
       for(int i=0;i<names.size()&&i<angles.size();i++)q["DH_"+names[i].toString().toStdString()]=angles[i].toDouble();
       if(QString(file).contains("grasp"))grasp_data=d;
     }
   }
   visualization_msgs::msg::MarkerArray markers;
   visualization_msgs::msg::Marker erase;erase.action=visualization_msgs::msg::Marker::DELETEALL;markers.markers.push_back(erase);
   auto tactile=grasp_data["tactile"].toObject();double tactile_age=hand_now-tactile["updated"].toDouble();auto fingers=tactile["fingers"].toArray();
   if(tactile_age>=0&&tactile_age<.08&&fingers.size()==5){
     const char* frames[]={"DH_finger13_link","DH_finger23_link","DH_finger33_link","DH_finger43_link","DH_finger53_link"};
     for(int i=0;i<5;i++){
       auto f=fingers[i].toObject();auto nf=f["normal_n"].toArray(),tf=f["tangent_n"].toArray(),dir=f["direction_deg"].toArray();
       double normal=0,tangent=0,angle=-1;for(auto v:nf)normal+=v.toDouble();
       for(int j=0;j<tf.size();j++)if(tf[j].toDouble()>tangent){tangent=tf[j].toDouble();angle=j<dir.size()&&!dir[j].isNull()?dir[j].toDouble(-1):-1;}
       auto base=[&](int id){visualization_msgs::msg::Marker m;m.header.frame_id=frames[i];m.header.stamp=state->now();m.ns="local_force_illustration";m.id=i*3+id;m.action=m.ADD;m.pose.orientation.w=1;m.lifetime=rclcpp::Duration::from_seconds(.08);return m;};
       auto arrow=[&](int id,double x,double y,double z){auto m=base(id);m.type=m.ARROW;m.scale.x=.002;m.scale.y=.004;m.scale.z=.006;m.color.a=.95;m.color.r=id==1?.95:.3;m.color.g=id==1?.65:.65;m.color.b=id==1?.25:1.;geometry_msgs::msg::Point origin,end;end.x=x;end.y=y;end.z=z;m.points={origin,end};markers.markers.push_back(m);};
       // Axes are an uncalibrated link-local illustration, explicitly labelled in the UI.
       // Never represent these directions as a measured world-frame force vector.
       if(normal>0&&std::isfinite(normal))arrow(0,0,0,std::min(.07,.008+normal*.008));
       if(tangent>0&&std::isfinite(tangent)&&angle>=0&&angle<=360){double len=std::min(.06,.008+tangent*.01),rad=angle*3.141592653589793/180;arrow(1,len*std::cos(rad),len*std::sin(rad),0);}
       auto text=base(2);text.type=text.TEXT_VIEW_FACING;text.scale.z=.009;text.color.r=.9;text.color.g=.9;text.color.b=.9;text.color.a=1;text.pose.position.z=.08;
       text.text="N "+QString::number(normal,'f',2).toStdString()+" / T "+QString::number(tangent,'f',2).toStdString()+" N (local*)";markers.markers.push_back(text);
     }
   }
   force_pub->publish(markers);
   QFile target_file(root+"/.cache/operator/target_state.json");QJsonObject target_data;
   if(target_file.open(QIODevice::ReadOnly))target_data=QJsonDocument::fromJson(target_file.readAll()).object();
   auto target_markers=target_display::make(target_data,hand_now,target_session_floor);
   Ogre::Vector3 base_position;Ogre::Quaternion base_orientation;
   bool base_valid=manager->getFrameManager()->getTransform(std::string("base_link"),base_position,base_orientation);
   target_axes->update(target_markers,base_position,base_orientation,base_valid);
   target_markers.markers.erase(std::remove_if(target_markers.markers.begin(),target_markers.markers.end(),[](const auto& m){return m.action==m.ADD&&m.id<20;}),target_markers.markers.end());
   target_pub->publish(target_markers);
   state->pose(q);rclcpp::spin_some(state);
   QFile sizefile(root+"/.cache/operator/scene_viewport.json");QJsonObject size;
   if(sizefile.open(QIODevice::ReadOnly))size=QJsonDocument::fromJson(sizefile.readAll()).object();
   int w=std::clamp(size["width"].toInt(1070),1,1600),h=std::clamp(size["height"].toInt(460),1,900);
   if(w!=width || h!=height){
     if(texture){target->removeAllViewports();Ogre::TextureManager::getSingleton().remove(texture->getHandle());texture.reset();}
     width=w;height=h;window.resize(w,h);manager->getRenderPanel()->resize(w,h);
     texture=Ogre::TextureManager::getSingleton().createManual("v3_canvas",Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,Ogre::TEX_TYPE_2D,w,h,0,Ogre::PF_BYTE_RGBA,Ogre::TU_RENDERTARGET);
     target=texture->getBuffer()->getRenderTarget();auto vp=target->addViewport(view->getCamera());vp->setBackgroundColour(background);vp->setOverlaysEnabled(false);target->setAutoUpdated(false);pixels.resize(w*h*4);
   }
   QFile events(root+"/.cache/operator/scene_input.json");if(events.open(QIODevice::ReadOnly)){
     for(auto value:QJsonDocument::fromJson(events.readAll()).array()){
       auto e=value.toObject();if(e["seq"].toDouble()<=input_seq)continue;input_seq=e["seq"].toDouble();
       auto pos=QPointF(e["x"].toDouble(),e["y"].toDouble());auto buttons=Qt::MouseButtons(e["buttons"].toInt());auto mods=Qt::KeyboardModifiers(e["mods"].toInt());
       if(e["kind"].toString()=="wheel"){
         QWheelEvent event(pos,pos,QPoint(),QPoint(0,e["delta"].toInt()),buttons,mods,Qt::NoScrollPhase,false);
         manager->handleMouseEvent(rviz_common::ViewportMouseEvent(manager->getRenderPanel(),&event,e["lx"].toInt(),e["ly"].toInt()));
       }else{
         auto type=e["kind"].toString()=="press"?QEvent::MouseButtonPress:e["kind"].toString()=="release"?QEvent::MouseButtonRelease:QEvent::MouseMove;
         QMouseEvent event(type,pos,Qt::MouseButton(e["button"].toInt()),buttons,mods);
         manager->handleMouseEvent(rviz_common::ViewportMouseEvent(manager->getRenderPanel(),&event,e["lx"].toInt(),e["ly"].toInt()));
       }
     }
   }
   view->getCamera()->setAutoAspectRatio(false);view->getCamera()->setAspectRatio(float(width)/height);
   target->update();Ogre::PixelBox box(width,height,1,Ogre::PF_BYTE_RGBA,pixels.data());target->copyContentsToMemory(Ogre::Box(0,0,width,height),box,Ogre::RenderTarget::FB_AUTO);
   header->seq=++sequence;__sync_synchronize();header->width=width;header->height=height;header->stride=width*4;
   header->updated=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();header->distance=view->subProp("Distance")->getValue().toDouble();
   std::memcpy(memory+64,pixels.data(),pixels.size());__sync_synchronize();header->seq=++sequence;


   if(qEnvironmentVariableIsSet("V3_SCENE_CAPTURE") && QDateTime::currentMSecsSinceEpoch()-window.property("capture_at").toLongLong()>1000){
     window.setProperty("capture_at",QDateTime::currentMSecsSinceEpoch());
     auto camera=view->getCamera();auto position=camera->getDerivedPosition();QJsonArray shown;for(int i=1;i<=6;i++)shown.append(q["joint_"+std::to_string(i)]);
     QFile report(root+"/artifacts/operator_scene_view.json");report.open(QIODevice::WriteOnly);
     report.write(QJsonDocument(QJsonObject{{"pid",int(getpid())},{"joint_radians",shown},{"distance",view->subProp("Distance")->getValue().toDouble()},{"x",position.x},{"y",position.y},{"z",position.z},{"width",width},{"height",height},{"offscreen",!window.isVisible()},{"frame",double(sequence)}}).toJson());

   }
 });timer.start(33);
 int rc=app.exec();manager->stopUpdate();target_axes.reset();munmap(memory,capacity);::close(fd);state.reset();abstraction.reset();rclcpp::shutdown();return rc;
}
