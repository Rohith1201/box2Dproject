/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/*
 * Base code for CS 251 Software Systems Lab
 * Department of Computer Science and Engineering, IIT Bombay
 *
 */


#include "cs251_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
#include<cmath>
using namespace std;

#include "dominos.hpp"


namespace cs251
{
  /**  The is the constructor
   * This is the documentation block for the constructor.
   */

  dominos_t::dominos_t()
  {
  #define DEGTORAD 0.0174532925199432957f
    //Ground
    /*! \var b1
     * \brief pointer to the body ground
     */
    b2Body* b1;
    {

      b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }

    //Top horizontal shelf
    {
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);

      b2BodyDef bd;
      bd.position.Set(-26.0f, 30.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);

       b2PolygonShape shape1;
      shape1.SetAsBox(0.2f, 1.5f);
      b2FixtureDef fd;
      fd.shape = &shape1;
      fd.density = 3.0f;
      fd.friction = 0.5f;
       b2BodyDef bd1;
	  bd1.type = b2_dynamicBody;
	  bd1.position.Set(-21.0f, 31.75f);
	  b2Body* body = m_world->CreateBody(&bd1);
	  body->CreateFixture(&fd);

	   b2RevoluteJointDef revoluteJointDef;
  revoluteJointDef.bodyA = ground;
  revoluteJointDef.bodyB = body;
  revoluteJointDef.collideConnected = false;
  revoluteJointDef.localAnchorA.Set(5.0f,0.0f);//the top right corner of the box
  revoluteJointDef.localAnchorB.Set(0.0f,-1.5f);//center of the circle
  m_world->CreateJoint( &revoluteJointDef );

    }

    //Dominos
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;

      for (int i = 0; i < 9; ++i)
	{
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-30.5f + 1.0f * i, 31.25f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
    }

    //Another horizontal shelf
    {
      b2PolygonShape shape;
      shape.SetAsBox(7.0f, 0.25f, b2Vec2(-24.5f,20.f), 0.0f);

      b2BodyDef bd;
      bd.position.Set(1.0f, 6.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

  // alarm shelf
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.0f, 0.2f);

      b2BodyDef bd;
      bd.position.Set(-32.0f, 10.0f);
      b2Body* alarm = m_world->CreateBody(&bd);
      alarm->CreateFixture(&shape, 0.0f);
      bd.position.Set(-32.0f, 13.4f);
      bd.type = b2_dynamicBody;
      shape.SetAsBox(0.8f, 0.2f);
      b2Body* alarmbox = m_world->CreateBody(&bd);
      alarmbox->CreateFixture(&shape, 0.0f);
      }
    //The pendulum that knocks the dominos off
    {
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 1.f);

	b2BodyDef bd;
	bd.position.Set(-31.5f, 29.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }

      b2Body* b4;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 0.25f);

	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(-35.0f, 33.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&shape, 2.0f);
      }

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-32.0f, 40.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }

    //The train of small spheres
    {
      b2Body* spherebody;
      //b2Body* spherebody1;

      b2CircleShape circle;
      circle.m_radius = 0.5;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 3.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;

      for (int i = 0; i<35; ++i)
	{
	  b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
	  if(i<10){
	  ballbd.position.Set(-30.f + i*1.0, 26.6f);
	   spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);
	  }
	  if(i==10){ballbd.position.Set(-6.8f, 9.3f);
      spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);
	  spherebody->SetGravityScale(-0.7);}
	  if(i==11){ballbd.position.Set(-6.8f, 20.7f);
	  ballfd.density = 20.0f;
      spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);}
	  if(i==12){ballbd.position.Set(-2.8f, 20.7f);
	  ballfd.density = 20.0f;
      spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);}
	  //ball falling on spring
      if(i==13){ballbd.position.Set(12.2f, 21.5f);
	  ballfd.density = 25.0f;circle.m_radius = 1.0;
      spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);}
	  //ball beside spring
	  if(i==14){ballbd.position.Set(22.0f, 13.0f);
	  ballfd.density = 25.0f;
	  ballfd.restitution = 0.5f;
      spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);}
	  if(i==15){ballbd.position.Set(-36.0f, 21.2f);
	  circle.m_radius = 1.0;
	  ballfd.density = 10.0f;
	  ballfd.restitution = 0.0f;
      spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);}
	  if(i==16){ballbd.position.Set(-32.0f, 11.8f);
	  circle.m_radius = 1.6;
	  ballfd.density = 10.0f;
	  ballfd.restitution = 0.0f;
      spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);}
	  if(i>16){
	  ballfd.restitution=0.0f;circle.m_radius = 0.5;
	   ballfd.density = 1.0f;
	   ballbd.position.Set(36.0+i*0.001,i-3.5);
	  ballfd.density = 30.0f;
      spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);
	  //ballbd.position.Set(32.0+i*0.001,i-1.5);
	  //spherebody1 = m_world->CreateBody(&ballbd);
	  //spherebody1->CreateFixture(&ballfd);
	  }
	}
    }

     //The pulley system below pendulum
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-20.0,10);
      bd->fixedRotation = true;

      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.0f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;

      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      //The bar
      bd->position.Set(-11.5,10);
      fd1->density = 40.0;
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

      // The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(-20.0, 10); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(-11.5, 10); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(-20.0, 16); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(-11.5, 16); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }

        //The revolving horizontal platform
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.8f, 0.2f);

      b2BodyDef bd;
      bd.type = b2_dynamicBody;

      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;


      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.8f);
      b2BodyDef bd2;

      for(int i=0;i<2;i++)
      { if(i==0){
        bd.position.Set(-6.8f, 10.0f);
        bd2.position.Set(-6.8f, 10.0f);
        }
        if(i==1){
        bd.position.Set(-4.8f, 20.0f);
        bd2.position.Set(-4.8f, 20.0f);
        }
        b2Body* body = m_world->CreateBody(&bd);
         body->CreateFixture(fd);
      b2Body* body2 = m_world->CreateBody(&bd2);
      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
      }

    }


    //The pulley system above spring
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(0,15);
      bd->fixedRotation = true;

      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.5f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-2.0f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.5f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.5f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;

      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);
      box1->SetGravityScale(0);

     //The bar
      b2PolygonShape bs;
       bs.SetAsBox(2.0f, 0.2f);
      fd1->shape = &bs;
      bd->position.Set(12,20);
      fd1->density = 10.0;
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);
      box2->SetGravityScale(0);

      // The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(0, 15); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(12, 20); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(0, 20); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(6, 20); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1,worldAnchorGround2,box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);

    }

 // surface
 for(int i=0;i<32;i++){
 float a,b,c,d;
 //bottom ege
 if(i==0){a=8.0f;b=12.0f;c=25.0f;d=12.0f;}
 if(i==1){a=25.0f;b=12.0f;c=25.0f;d=8.0f;}
 if(i==2){a=25.0f;b=8.0f;c=29.0f;d=8.0f;}
 if(i==3){a=29.0f;b=8.0f;c=29.0f;d=12.0f;}
 if(i==4){a=29.0f;b=12.2f;c=42.0f;d=12.0f;}
 if(i==5){a=42.0f;b=12.0f;c=42.0f;d=44.f;}
 //ball falling on spring
 if(i==6){a=10.1f;b=20.2f;c=10.1f;d=25.0f;}
 if(i==7){a=13.9f;b=20.2f;c=13.9f;d=25.0f;}
 if(i==8){a=10.1f;b=19.8f;c=11.1f;d=18.6f;}
 if(i==9){a=13.9f;b=19.8f;c=13.9f;d=19.6f;}
 //balls
 if(i==10){a=35.0f;b=12.0f;c=35.0f;d=30.0f;}
 if(i==11){a=37.8f;b=13.5f;c=37.8f;d=30.0f;}
 //bar
 if(i==12){a=38.22f;b=13.2f;c=38.22f;d=15.0f;}
 //spring
 if(i==13){a=10.5f;b=12.f;c=10.5f;d=16.0f;}
 //lifting part
 if(i==14){a=40.f;b=13.4f;c=40.f;d=41.8f;}
 if(i==15){a=40.f;b=41.8f;c=36.f;d=41.8f;}
 // top
 if(i==16){a=8.f;b=39.f;c=14.2f;d=39.f;}
 if(i==17){a=8.f;b=38.f;c=8.f;d=39.f;}
 if(i==18){a=6.f;b=38.f;c=8.f;d=38.f;}
 if(i==19){a=6.f;b=39.f;c=6.f;d=38.f;}
 if(i==20){a=-10.f;b=39.f;c=6.f;d=39.f;}
 if(i==21){a=-10.f;b=36.f;c=-10.f;d=39.f;}
 if(i==22){a=-12.3f;b=36.f;c=-10.f;d=36.f;}
 if(i==23){a=-12.3f;b=39.f;c=-12.3f;d=36.f;}
 if(i==24){a=-35.4f;b=39.f;c=-12.3f;d=39.f;}
 if(i==25){a=-36.5f;b=35.f;c=-36.5f;d=42.f;}
 //left side
  if(i==26){a=-37.2f;b=19.8f;c=-37.2f;d=19.2f;}
   if(i==27){a=-34.8f;b=19.8f;c=-34.8f;d=19.2f;}
   //shelf
   if(i==28){a=-32.0f;b=25.0f;c=-32.0f;d=17.f;}
   if(i==29){a=-32.f;b=17.f;c=-21.f;d=17.f;}
   //alarm
   if(i==30){a=-33.f;b=10.2f;c=-32.7f;d=10.5f;}
   if(i==31){a=-31.f;b=10.2f;c=-31.3f;d=10.5f;}
  b2Body* b1;
    {
      b2EdgeShape shape;
      shape.Set(b2Vec2(a,b), b2Vec2(c,d));
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->shape = &shape;
      if(i==28){fd1->restitution =1.0;}
      else{fd1->restitution =0.0;}
      fd1->friction =1;
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(fd1);
    }
   }

    //The pulley system near wheels
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(27.0,11);
      bd->fixedRotation = true;

      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.0;
      fd1->restitution = 0.0f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs;
      bs.SetAsBox(2.0f, 0.1f);
      fd1->shape = &bs;

      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);

      //The bar
      bd->position.Set(38,14.0);
      fd1->density = 5.0;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape s;
      s.SetAsBox(0.2f, 2.0f);
      fd1->shape = &s;
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

      // The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(27, 11); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(38, 14.0); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(27, 24); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(38, 24); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }


    //bigpulley on left side
    {
     //The pulley system

      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-36,30);
      bd->fixedRotation = true;

      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 0.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.0f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 0.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(1.5f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-1.5f,0.f), 0);
      fd3->shape = &bs3;

      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      //The bar
      bd->position.Set(-42,25);
    //  fd1->density = 40.0;
      b2Body* box2 = m_world->CreateBody(bd);
      //box2->CreateFixture(fd1);

      // The pulley joint fd1- fd1->density = 40.0;>density = 40.0;
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(-36, 30); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(-42, 25); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(-36, 35); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(-42, 35); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);

     //The pulley system
      bd->position.Set(-36,20);
     //The bar
      b2PolygonShape bs;
       bs.SetAsBox(1.6f, 0.2f);
      fd1->shape = &bs;
      fd1->friction = 0.0;
      fd1->density = 0.0;
      b2Body* box3 = m_world->CreateBody(bd);
      box3->CreateFixture(fd1);
      box3->SetGravityScale(0);

      // The pulley joint
      b2PulleyJointDef* myjoint1 = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody3(-42, 25); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody4(-36, 20); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround3(-42, 20); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround4(-41.9, 20); // Anchor point for ground 2 in world axis
     // float32 ratio = 1.0; // Define ratio
      myjoint1->Initialize(box2, box3, worldAnchorGround3,worldAnchorGround4,box2->GetWorldCenter(), box3->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint1);
    }


    //spring
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.5f, 0.2f);

      b2BodyDef bd1,bd2,bd3,bd4;
      bd1.type = b2_dynamicBody;
      bd2.type = b2_dynamicBody;
      bd3.type = b2_dynamicBody;
      bd4.type = b2_dynamicBody;

      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.2f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      fd->friction = 4;
      fd->restitution = 0.0f;
        bd1.position.Set(12.2f, 14.0f);
        bd2.position.Set(12.2f, 14.0f);
        bd3.position.Set(15.2f, 14.0f);
        bd4.position.Set(15.2f, 14.0f);
        bd1.angle=53*DEGTORAD;
        bd2.angle=-53*DEGTORAD;
        bd3.angle=53*DEGTORAD;
        bd4.angle=-53*DEGTORAD;

        b2Body* body1 = m_world->CreateBody(&bd1);
        b2Body* body2 = m_world->CreateBody(&bd2);
        b2Body* body3 = m_world->CreateBody(&bd3);
        b2Body* body4 = m_world->CreateBody(&bd4);
         body1->CreateFixture(fd);
         body2->CreateFixture(fd);
         body3->CreateFixture(fd);
         body4->CreateFixture(fd);
        /* body1->SetGravityScale(0);
         body2->SetGravityScale(0);
         body3->SetGravityScale(0);
         body4->SetGravityScale(0);*/
      b2RevoluteJointDef jointDef1;
      jointDef1.bodyA = body1;
      jointDef1.bodyB = body4;
      //jointDef1.maxLength=0.10001f;
      jointDef1.localAnchorA.Set(2.5,0);
      jointDef1.localAnchorB.Set(-2.5,0);
      jointDef1.collideConnected = true;
      m_world->CreateJoint(&jointDef1);
      b2RevoluteJointDef jointDef2;
      jointDef2.bodyA = body2;
      jointDef2.bodyB = body3;
      //jointDef2.maxLength=0.10001f;
      jointDef2.localAnchorA.Set(2.5,0);
      jointDef2.localAnchorB.Set(-2.5,0);
      jointDef2.collideConnected = true;
      m_world->CreateJoint(&jointDef2);
      b2RevoluteJointDef jointDef3;
      jointDef3.bodyA = body1;
      jointDef3.bodyB = body2;
      //jointDef3.maxLength=0.00001f;
      jointDef3.localAnchorA.Set(0,0);
      jointDef3.localAnchorB.Set(0,0);
      jointDef3.collideConnected = false;
      m_world->CreateJoint(&jointDef3);
      b2RevoluteJointDef jointDef4;
      jointDef4.bodyA = body3;
      jointDef4.bodyB = body4;
      //jointDef4.maxLength=0.00001f;
      jointDef4.localAnchorA.Set(0,0);
      jointDef4.localAnchorB.Set(0,0);
      jointDef4.collideConnected = false;
      m_world->CreateJoint(&jointDef4);
      }

      //conveyor belt
      {
      b2Body *s11,*s12,*s13,*s14,*s21,*s22,*s23,*s24,*sb11,*sb13,*s1,*s2;

      b2PolygonShape bs;
       bs.SetAsBox(0.5f, 0.5f);
      b2FixtureDef ballfd;
      ballfd.shape = &bs;
      ballfd.density = 3.0f;
      ballfd.friction = 0.2f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
      ballbd.position.Set(24.f, 41.0f);sb11 = m_world->CreateBody(&ballbd);sb11->CreateFixture(&ballfd);sb11->SetAngularVelocity(0);
      ballbd.position.Set(24.f, 35.0f);sb13 = m_world->CreateBody(&ballbd);sb13->CreateFixture(&ballfd);sb13->SetAngularVelocity(0);
      //ballbd.position.Set(9.f, 35.0f);sb21 = m_world->CreateBody(&ballbd);sb21->CreateFixture(&ballfd);
      //ballbd.position.Set(9.f, 29.0f);sb23 = m_world->CreateBody(&ballbd);sb23->CreateFixture(&ballfd);
       s11=sb11;
       s13=sb13;
       s21 =sb11;

      for (int i = 0; i<19; ++i)
	{

	  if(i<19){
	  ballbd.position.Set(18.f+i*1.0f, 41.0f);s12 = m_world->CreateBody(&ballbd);s12->CreateFixture(&ballfd);s12->SetAngularVelocity(0);
      ballbd.position.Set(18.f+i*1.0f, 35.0f);s14 = m_world->CreateBody(&ballbd);s14->CreateFixture(&ballfd);s14->SetAngularVelocity(0);
      b2RopeJointDef jointDef11;jointDef11.bodyA = s11;jointDef11.bodyB = s12;jointDef11.localAnchorA.Set(0,0);
	   jointDef11.localAnchorB.Set(0,0);jointDef11.maxLength=1.f;jointDef11.collideConnected = false;m_world->CreateJoint(&jointDef11);
        s11=s12;
	   b2RopeJointDef jointDef21;jointDef21.bodyA = s13;jointDef21.bodyB = s14;jointDef21.localAnchorA.Set(0,0);
	   jointDef21.localAnchorB.Set(0,0);jointDef21.maxLength=1.f;jointDef21.collideConnected = false;m_world->CreateJoint(&jointDef21);
	   s13=s14;
	  }
	  }
	   s23=s11;
	  for (int i =10; i<16; ++i){
	  if(9<i && i<19){
	  ballbd.position.Set(18.0f-3.0f*sin((i-9)*30*DEGTORAD),38.0f+3.0f*cos((i-9)*30*DEGTORAD));s22 = m_world->CreateBody(&ballbd);s22->CreateFixture(&ballfd);s22->SetAngularVelocity(0);
      ballbd.position.Set(36.0f+3.0f*sin((i-9)*30*DEGTORAD),38.0f+3.0f*cos((i-9)*30*DEGTORAD));s24 = m_world->CreateBody(&ballbd);s24->CreateFixture(&ballfd);s24->SetAngularVelocity(0);
      b2RopeJointDef jointDef11;jointDef11.bodyA = s21;jointDef11.bodyB = s22;jointDef11.localAnchorA.Set(0,0);
	   jointDef11.localAnchorB.Set(0,0);jointDef11.maxLength=1.f;jointDef11.collideConnected = false;m_world->CreateJoint(&jointDef11);
        s21=s22;
	   b2RopeJointDef jointDef21;jointDef21.bodyA = s23;jointDef21.bodyB = s24;jointDef21.localAnchorA.Set(0,0);
	   jointDef21.localAnchorB.Set(0,0);jointDef21.maxLength=1.f;jointDef21.collideConnected = false;m_world->CreateJoint(&jointDef21);
	   s23=s24;
	  }
	  }
	  b2RopeJointDef jointDef11;jointDef11.bodyA = s21;jointDef11.bodyB = sb13;jointDef11.localAnchorA.Set(0,0);
	   jointDef11.localAnchorB.Set(0,0);jointDef11.maxLength=1.f;jointDef11.collideConnected = false;m_world->CreateJoint(&jointDef11);
	   b2RopeJointDef jointDef21;jointDef21.bodyA = s23;jointDef21.bodyB = s13;jointDef21.localAnchorA.Set(0,0);
	   jointDef21.localAnchorB.Set(0,0);jointDef21.maxLength=1.f;jointDef21.collideConnected = false;m_world->CreateJoint(&jointDef21);

      ballbd.type = b2_kinematicBody;
      b2CircleShape circle;
      circle.m_radius = 2.5;
      ballfd.shape = &circle;
      ballfd.friction = 1.f;
      ballbd.position.Set(18.f, 38.0f);s1 = m_world->CreateBody(&ballbd);s1->CreateFixture(&ballfd);
      ballbd.position.Set(36.f, 38.0f);s2 = m_world->CreateBody(&ballbd);s2->CreateFixture(&ballfd);
      s1->SetAngularVelocity(1.7);
      s2->SetAngularVelocity(1.7);
      // box
      b2PolygonShape shape;
      shape.SetAsBox(1.0f, 1.0f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 10.0f;
      fd.friction = 1.0f;
      fd.restitution = 0.0f;
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(19.5f, 13.0f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
    }

     // lifting
     {
      b2PolygonShape shape;
      shape.SetAsBox(1.0f, 0.1f);

      b2BodyDef bd1,bd2;
      bd1.type = b2_kinematicBody;
      bd2.type = b2_kinematicBody;
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.2f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      fd->friction = 1;
      fd->restitution = 0.0f;
      for(int i=0;i<15;i++){
        bd1.position.Set(41.0f, 13.5f+i*2.0f);
        bd2.position.Set(41.0f, 13.5f+i*2.0f);
        bd1.angle=i*DEGTORAD;
        bd2.angle=(90+i)*DEGTORAD;
        b2Body* body1 = m_world->CreateBody(&bd1);
        b2Body* body2 = m_world->CreateBody(&bd2);
         body1->CreateFixture(fd);
         body2->CreateFixture(fd);

      b2RevoluteJointDef jointDef1;
      jointDef1.bodyA = body1;
      jointDef1.bodyB = body2;
      jointDef1.localAnchorA.Set(0,0);
      jointDef1.localAnchorB.Set(0,0);
      jointDef1.collideConnected = false;
      m_world->CreateJoint(&jointDef1);
      body1->SetAngularVelocity(3);
      body2->SetAngularVelocity(3);
      }
     }

     // wedge - seesaw system
     {
      //The triangle wedge
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,1.5);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-40.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);

      //The plank on top of the wedge
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.2f);
      b2BodyDef bd2;
      bd2.position.Set(-40.0f, 1.5f);
      bd2.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.f;
      fd2->friction=1.0;
      fd2->restitution=0.0;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape;
      body->CreateFixture(fd2);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-40.0f, 1.5f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);

      //The light box on the right side of the see-saw
      b2PolygonShape shape2;
      shape2.SetAsBox(1.0f, 1.0f);
      b2BodyDef bd3;
      bd3.position.Set(-43.5f, 2.0f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 1.0f;fd3->friction=1.0;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
    }
//skksm
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
