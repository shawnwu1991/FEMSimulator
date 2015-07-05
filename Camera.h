#pragma once

#include <QtCore\QVector>
#include "comHeader.h"

class Camera final
{
public:
    Camera(void);
    ~Camera(void);

public:
    const QMatrix4x4 &GetViewMatrix(void) const { return m_viewMatrix; }
    const QMatrix4x4 &GetProjectMatrix(void) const { return m_projectMatrix; }
    const QMatrix4x4 &GetProjViewMatrix(void) const { return m_projViewMatrix; }

    void ResetView(void);

    void LookAt(const QVector3D &eye, const QVector3D &center,const QVector3D &up);

    void Perspective(float fov, float aspect, float nearPlane, float farPlane);
    void SetAspectRatio(float ratio) { Perspective(m_fov, ratio, m_nearPlane, m_farPlane); }
    void SetAspectRatio(float width, float height) { Perspective(m_fov, width/height, m_nearPlane, m_farPlane); }

	const QVector3D &GetCameraEye() const { return m_eye; } 
	const QVector3D &GetCameraCenter() const { return m_center; } 
	const QVector3D &GetCameraUp() const { return m_up; } 
	const float &GetCameraFov() const { return m_fov; } 
	const float &GetCameraAspect() const { return m_aspectRatio; } 
	const float &GetCameraNearPlane() const { return m_nearPlane; } 
	const float &GetCameraFarPlane() const { return m_farPlane; } 

public:
    enum DragType { Drag_Rotate, Drag_Move, Drag_Zoom };
    void OnMousePressEvent(QVector2D pt);
    void OnMouseReleaseEvent(QVector2D pt);
    void OnMouseMoveEvent(QVector2D pt, QVector2D oldpt, DragType dragType);

private:
    void CaptureState(const QVector2D &pt);
    void ReleaseState(void);
    QVector3D UnProjectToShpere(QVector2D pt);
    QVector3D UnProjectToPlane(QVector2D pt);

    void Rotate(const QVector2D &pt);
    void Move(const QVector2D &pt, const QVector2D &oldpt);
    void Zoom(const QVector2D &pt, const QVector2D &oldpt);

private:
    QVector3D m_eye;
    QVector3D m_center;
    QVector3D m_up;

    float m_fov;
    float m_aspectRatio;
    float m_nearPlane;
    float m_farPlane;
    
    QMatrix4x4 m_projectMatrix;
    QMatrix4x4 m_viewMatrix;
    QMatrix4x4 m_projViewMatrix;

    struct CameraState
    {
        QVector3D eye;
        QVector3D center;
        QVector3D up;

        float eyeCenterDist;
        float sphereRadius;

        QMatrix4x4 projViewMatrix;
        QMatrix4x4 invProjViewMatrix;

        QVector3D refPos;
        QVector2D refPoint;
    } m_state;
};

