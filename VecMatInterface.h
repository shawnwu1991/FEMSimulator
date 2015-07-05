#pragma once

#ifndef _VecMat_Interface_h_
#define _VecMat_Interface_h_

#include "Wm5\Wm5Vector2.h"
#include "Wm5\Wm5Vector3.h"
#include "Wm5\Wm5Vector4.h"
#include "Wm5\Wm5Quaternion.h"
#include "Wm5\Wm5Math.h"
#include "Wm5\Wm5Matrix2.h"
#include "Wm5\Wm5Matrix3.h"
#include "Wm5\Wm5Matrix4.h"
#include "Wm5\Wm5Math.h"

// core geometries
#include "Wm5\Wm5AxisAlignedBox2.h"
#include "Wm5\Wm5AxisAlignedBox3.h"
#include "Wm5\Wm5Box3.h"
#include "Wm5\Wm5IntrLine3Box3.h"
#include "Wm5\Wm5Sphere3.h"
#include "Wm5\Wm5IntrLine3Sphere3.h"
#include "Wm5\Wm5Ellipsoid3.h"
#include "Wm5\Wm5IntrLine3Ellipsoid3.h"
#include "Wm5\Wm5Plane3.h"
#include "Wm5\Wm5IntrLine3Plane3.h"
#include "Wm5\Wm5Capsule3.h"
#include "Wm5\Wm5IntrLine3Capsule3.h"

#include "Wm5\Wm5IntrBox3Box3.h"

//namespace SIMCON
//{
    typedef Wm5::Vector2d Vector2d;
    typedef Wm5::Vector3d Vector3d;
    typedef Wm5::Vector4d Vector4d;
    typedef Wm5::Vector2f Vector2f;
    typedef Wm5::Vector3f Vector3f;
    typedef Wm5::Vector4f Vector4f;
    typedef Wm5::Quaterniond Quaterniond;
    typedef Wm5::Matrix2d Matrix2d;
    typedef Wm5::Matrix3d Matrix3d;
    typedef Wm5::Matrix4d Matrix4d;
    typedef Wm5::Matrix3f Matrix3f;
    typedef Wm5::Matrix4f Matrix4f;
    typedef Wm5::AxisAlignedBox2d AxisAlignedBox2d;
    typedef Wm5::AxisAlignedBox3d AxisAlignedBox3d;
    typedef Wm5::Mathd Mathd;

    typedef Wm5::Segment3d Segment3d;
    typedef Wm5::Line3d Line3d;
    // core geometries
    typedef Wm5::Box3d Box3d;
    typedef Wm5::Sphere3d Sphere3d;
    typedef Wm5::Ellipsoid3d Ellipsoid3d;
    typedef Wm5::Plane3d Plane3d;
    typedef Wm5::Capsule3d Capsule3d;
    // intersection
    typedef Wm5::IntrLine3Box3d IntrLine3Box3d;
    typedef Wm5::IntrLine3Sphere3d IntrLine3Sphere3d;
    typedef Wm5::IntrLine3Ellipsoid3d IntrLine3Ellipsoid3d;
    typedef Wm5::IntrLine3Plane3d IntrLine3Plane3d;
    typedef Wm5::IntrLine3Capsule3d IntrLine3Capsule3d;

    // collision between bounding boxes
    typedef Wm5::IntrBox3Box3d IntrBox3Box3d;
//}

#endif