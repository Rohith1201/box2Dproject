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
using namespace std;

#include "dominos.hpp"

namespace cs251
{
  /**  The is the constructor
   * This is the documentation block for the constructor.
   */

  dominos_t::dominos_t()
  {
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

      b2CircleShape circle;
      circle.m_radius = 0.5;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 3.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;

      for (int i = 0; i<13; ++i)
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
	  ballfd.density = 8.0f;
      spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);}
	  if(i==12){ballbd.position.Set(-2.8f, 20.7f);
	  ballfd.density = 8.0f;
      spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);}
	}
    }

     //The pulley system
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-31.5,10);
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
      b2Vec2 worldAnchorOnBody1(-31.5, 10); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(-11.5, 10); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(-31.5, 16); // Anchor point for ground 1 in world axis
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


  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
