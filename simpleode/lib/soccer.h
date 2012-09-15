#ifdef soccer_h
#  error DO NOT INCLUDE soccer.h TWICE
#endif

#define soccer_h

#include "xode.h"

namespace xode
{

void TDynRobot::Create(dWorldID world, dSpaceID space)
{
  Clear();

  body_.resize(3);
  link_b_.resize(6);
  link_cy_.resize(1);

  int i(0),ib(0),icy(0);
  {
    dReal side1(0.1),side2(1.5),side3(0.5);
    link_b_[ib].create (space,side1,side2,side3);
    link_b_[ib].setPosition (1.0,0.0,0.25);
    ++ib;
  }
  {
    dReal side1(0.1),side2(1.5),side3(0.5);
    link_b_[ib].create (space,side1,side2,side3);
    link_b_[ib].setPosition (-1.0,0.0,0.25);
    ++ib;
  }
  {
    dReal side1(2.0),side2(0.1),side3(0.5);
    link_b_[ib].create (space,side1,side2,side3);
    link_b_[ib].setPosition (0.0,0.75,0.25);
    ++ib;
  }
  {
    dReal side1(2.0),side2(0.1),side3(0.5);
    link_b_[ib].create (space,side1,side2,side3);
    link_b_[ib].setPosition (0.0,-0.75,0.25);
    ++ib;
  }

  {
    dReal side1(0.2),side2(0.4),side3(0.3);
    body_[i].create (world);
    body_[i].setPosition (0.5,0.0,0.15);
    dMass m;
    m.setBox (1,side1,side2,side3);
    // m.adjust (mass);
    body_[i].setMass (&m);
    link_b_[ib].create (space,side1,side2,side3);
    link_b_[ib].setBody (body_[i]);
    ++i;++ib;
  }
  {
    dReal side1(0.2),side2(0.4),side3(0.3);
    body_[i].create (world);
    body_[i].setPosition (-0.5,0.0,0.15);
    dMass m;
    m.setBox (1,side1,side2,side3);
    // m.adjust (mass);
    body_[i].setMass (&m);
    link_b_[ib].create (space,side1,side2,side3);
    link_b_[ib].setBody (body_[i]);
    ++i;++ib;
  }

  // ball
  {
    body_[i].create (world);
    body_[i].setPosition (0.0,0.0,0.10);
    dMass m;
    dMassSetCylinder (&m,1,/*z*/3,/*radius*/0.1,/*length*/0.2);
    // m.adjust (mass);
    body_[i].setMass (&m);
    link_cy_[icy].create (space,/*radius*/0.1,/*length*/0.2);
    link_cy_[icy].setBody (body_[i]);
    ++i;++icy;
  }

}

}  // end of xode
