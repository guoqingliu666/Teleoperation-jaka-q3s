#pragma once
#include "target_markers.hpp"
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreEntity.h>
#include <OgreMaterial.h>
#include <rviz_rendering/objects/arrow.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/objects/movable_text.hpp>

// Reuse RViz's native shapes; render the target handles on top of the robot so
// the axes remain legible when the target coincides with the hand/TCP.
class TargetOverlay {
  Ogre::SceneManager* manager_;Ogre::SceneNode* node_;
  std::array<std::unique_ptr<rviz_rendering::Arrow>,3> arrows_;
  std::unique_ptr<rviz_rendering::Shape> center_;
  std::array<std::unique_ptr<rviz_rendering::MovableText>,4> labels_;
  std::array<Ogre::SceneNode*,4> label_nodes_;
  static void on_top(rviz_rendering::Shape* shape) {
    auto material=shape->getMaterial();material->setDepthCheckEnabled(false);material->setDepthWriteEnabled(false);
    shape->getEntity()->setRenderQueueGroup(95);
  }
public:
  explicit TargetOverlay(Ogre::SceneManager* manager):manager_(manager) {
    node_=manager->getRootSceneNode()->createChildSceneNode();
    for(int i=0;i<3;i++){arrows_[i]=std::make_unique<rviz_rendering::Arrow>(manager,node_,.102,.004,.018,.008);on_top(arrows_[i]->getHead());on_top(arrows_[i]->getShaft());}
    center_=std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere,manager,node_);on_top(center_.get());
    for(int i=0;i<4;i++){
      labels_[i]=std::make_unique<rviz_rendering::MovableText>(i==3?"TARGET":std::string(1,"XYZ"[i]),"Liberation Sans",i==3?.018:.022);
      labels_[i]->showOnTop();labels_[i]->setRenderQueueGroup(95);label_nodes_[i]=node_->createChildSceneNode();label_nodes_[i]->attachObject(labels_[i].get());
    }
    node_->setVisible(false);
  }
  ~TargetOverlay(){for(auto& l:labels_)l.reset();for(auto& a:arrows_)a.reset();center_.reset();node_->removeAndDestroyAllChildren();manager_->destroySceneNode(node_);}
  void update(const visualization_msgs::msg::MarkerArray& markers,const Ogre::Vector3& p,const Ogre::Quaternion& q,bool tf_valid) {
    node_->setVisible(false);if(!tf_valid)return;node_->setPosition(p);node_->setOrientation(q);
    for(const auto& m:markers.markers) {
      if(m.action!=target_display::Marker::ADD)continue;
      auto color=Ogre::ColourValue(m.color.r,m.color.g,m.color.b,m.color.a);
      auto pos=[](const auto& v){return Ogre::Vector3(v.x,v.y,v.z);};
      if(m.id<3&&m.type==target_display::Marker::ARROW){
        node_->setVisible(true);arrows_[m.id]->setPosition(pos(m.points[0]));arrows_[m.id]->setDirection(pos(m.points[1])-pos(m.points[0]));arrows_[m.id]->setColor(color);
        on_top(arrows_[m.id]->getHead());on_top(arrows_[m.id]->getShaft());
      }else if(m.id==3){center_->setPosition(pos(m.pose.position));center_->setScale(Ogre::Vector3(.014));center_->setColor(color);on_top(center_.get());}
      else if(m.id>=4&&m.id<=7){labels_[m.id-4]->setColor(color);label_nodes_[m.id-4]->setPosition(pos(m.pose.position));}
    }
  }
};
