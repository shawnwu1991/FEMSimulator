#include "stdafx.h"
#include "Camera.h"


Camera::Camera(void)
    : m_eye(3.0f, 2.0f, 3.0f)
    , m_center(0.0f, 0.0f, 0.0f)
    , m_up(0.0f, 1.0f, 0.0f)
    , m_fov(30.0f)
    , m_nearPlane(0.1f)
    , m_farPlane(100.0f)
    , m_aspectRatio(1.0f)
{
    m_projectMatrix.setToIdentity();
    m_projectMatrix.perspective(m_fov, 1.0f, m_nearPlane, m_farPlane);
    ResetView();
}


Camera::~Camera(void)
{
}


void Camera::ResetView(void)
{
    m_eye = QVector3D(3.0f, 2.0f, 3.0f);
    m_center = QVector3D(0.0f, 0.0f, 0.0f);
    m_up = QVector3D(0.0f, 1.0f, 0.0f);
    
    m_viewMatrix.setToIdentity();

    m_viewMatrix.lookAt(m_eye, m_center, m_up);
    m_projViewMatrix = m_projectMatrix * m_viewMatrix;
}


void Camera::Perspective(float fov, float aspect, float nearPlane, float farPlane)
{
    m_fov = fov;
    m_aspectRatio = aspect;
    m_nearPlane = nearPlane;
    m_farPlane = farPlane;

    m_projectMatrix.setToIdentity();
    m_projectMatrix.perspective(fov, aspect, nearPlane, farPlane);    
    m_projViewMatrix = m_projectMatrix * m_viewMatrix;
}

void Camera::LookAt(const QVector3D &eye, const QVector3D &center,const QVector3D &up)
{
    m_eye = eye;
    m_center = center;
    m_up = up;

    m_viewMatrix.setToIdentity();
    m_viewMatrix.lookAt(m_eye, m_center, m_up);
    m_projViewMatrix = m_projectMatrix * m_viewMatrix;
}

inline static QVector3D UnProject(QVector2D pt, const QMatrix4x4 &invProjViewMat)
{
    pt *= 2.0f;
    pt -= QVector2D(1.0f, 1.0f);
    QVector4D v(pt, -1.0f, 1.0f);
    v = invProjViewMat * v;
    QVector3D pos(v);
    pos /= v.w();
    return pos;
}


QVector3D Camera::UnProjectToShpere(QVector2D pt)
{
    QVector3D pos = UnProject(pt, m_state.invProjViewMatrix);
    
    QVector3D dir = pos - m_state.eye;
    
    // mouse position represents ray: eye + t*dir
    // intersecting with a sphere centered at the origin
    float a = QVector3D::dotProduct(dir, dir);
    float b = 2.0f * QVector3D::dotProduct(m_state.eye - m_state.center, dir);
    float c = m_state.eyeCenterDist * m_state.eyeCenterDist - m_state.sphereRadius * m_state.sphereRadius;
    float delta = b * b - 4.0f * a * c;
    if(delta < 0) 
        delta = 0;
    else
        delta = sqrt(delta);

    float t = (-b - delta) / 2.0 / a;
    return m_state.eye + dir * t;
}

QVector3D Camera::UnProjectToPlane(QVector2D pt)
{
    QVector3D pos = UnProject(pt, m_state.invProjViewMatrix);

    return pos;
}

void Camera::OnMousePressEvent(QVector2D pt)
{
    CaptureState(pt);
}

void Camera::OnMouseReleaseEvent(QVector2D pt)
{
    ReleaseState();
}

void Camera::OnMouseMoveEvent(QVector2D pt, QVector2D oldpt, DragType dragType)
{
    switch (dragType)
    {
    case Drag_Rotate:
        Rotate(pt);
        break;

    case Drag_Move:
        Move(pt, oldpt);
        break;

    case Drag_Zoom:
        Zoom(pt, oldpt);
        break;
    }
}

void Camera::CaptureState(const QVector2D &pt)
{
    m_state.eye = m_eye;
    m_state.center = m_center;    
    m_state.up = m_up;

    m_state.eyeCenterDist = (m_eye - m_center).length();
    float ratio = sqrt(1.0f + m_aspectRatio * m_aspectRatio);
    float angle = m_fov / 2.0f * M_PI / 180.0f;
    m_state.sphereRadius = ratio * m_state.eyeCenterDist * tan(angle) / 2.0f;

    m_state.projViewMatrix = m_projViewMatrix;
    m_state.invProjViewMatrix = m_projViewMatrix.inverted();

    m_state.refPoint = pt;
    m_state.refPos = UnProjectToShpere(pt);
}

void Camera::ReleaseState(void)
{
}

void Camera::Rotate(const QVector2D &pt)
{
    QVector3D pos = UnProjectToShpere(pt);
    //printf("%0.6f %0.6f %0.6f\n", pos.x(), pos.y(), pos.z());
    QVector3D v0 = (m_state.refPos - m_state.center).normalized();
    QVector3D v1 = (pos - m_state.center).normalized();

    QVector3D axis = QVector3D::crossProduct(v0, v1).normalized();
    float angle = acos(QVector3D::dotProduct(v0, v1)) / M_PI * 180.0f;

    QQuaternion qt = QQuaternion::fromAxisAndAngle(axis, angle);
    QVector3D newEye = qt.conjugate().rotatedVector(m_state.eye - m_state.center) + m_state.center;
    LookAt(newEye, m_state.center, m_state.up);
}

void Camera::Move(const QVector2D &pt, const QVector2D &oldpt)
{
    float offx = pt.x() - oldpt.x();
    float offy = pt.y() - oldpt.y();
    //const float maxZoomStep = 1.0f;
    //const float minZoomStep = 0.1f;
    const float scaleX = 1.0f * m_state.eyeCenterDist;
    const float scaleY = -1.0f * scaleX / m_aspectRatio;
    //const float scaleY = -2.0f * scaleX / m_aspectRatio;

    auto toCenter = m_center - m_eye;
    auto toLeft = QVector3D::crossProduct(m_up, toCenter).normalized();
    auto toUp = QVector3D::crossProduct(toCenter, toLeft).normalized();

    auto offset = (offx * scaleX) * toLeft + (offy * scaleY) * toUp;
    LookAt(m_eye + offset, m_center + offset, m_up);
}

void Camera::Zoom(const QVector2D &pt, const QVector2D &oldpt)
{
    float off = pt.y() - oldpt.y();
    //const float maxZoomStep = 1.0f;
    //const float minZoomStep = 0.1f;
    const float scale = 4.0f;
    const float minDist = 0.1f;

    float dist = (m_eye - m_center).length();
    float newDist = dist - scale * off;
    if (newDist < 0.1f)
        newDist = 0.1f;

    double offRatio = newDist / dist;
    LookAt(m_center + (m_eye - m_center) * offRatio, m_center, m_up);
}

