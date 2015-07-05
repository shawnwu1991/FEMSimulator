#pragma once

#include <QtCore\QVector>
#include <QtOpenGL\QGLFunctions>
#include <QtOpenGL\QGLShaderProgram>

#include "comHeader.h"
#include "VecMatInterface.h"
#include "TetMesh.h"

class CentralGLWidget;
class GLTetMesh final: protected QGLFunctions
{
public:
    GLTetMesh(CentralGLWidget *glWidget, std::shared_ptr<Libin::TetMesh> tetMesh);
    ~GLTetMesh(void);

public:
    void UpdateVertexNormal(void);
	void UpdateSurfaceArea(void);
    void SetVertForce(const double *vertForce, double scale) { m_vertForce = vertForce; m_forceScale = (float) scale; }

    void DrawTetMesh(void);
    void DrawVertForce(void);
    void DrawVertices(void);
 
    void OnSetForceScaleLevel(int scalelevel);

	std::vector<double>& GetSurfaceArea() { return m_surfaceArea; }
	QVector<GLint>& GetSurfaceFace() { return m_surfaceFace; }
	QVector<QVector3D>& GetSurfaceNormal() { return m_surfaceNormals; }
	std::vector<int>& GetSurfaceVertexMask() { return m_surfaceVertMask; }
	std::shared_ptr<Libin::TetMesh> GetTetmesh() { return m_tetMesh; }

	int GetSurfaceVertices(std::vector<float>& buf);
	int GetSurfaceFaces(std::vector<uint>& buf);private:
    void Initialize(void);
    void InitializeShaders(void);

private:
    std::shared_ptr<Libin::TetMesh> m_tetMesh;

    CentralGLWidget *m_glWidget;
    QGLShaderProgram m_surfaceShader;
    QGLShaderProgram m_forceShader;
    
    QVector<QVector3D> m_surfaceVertices;
    QVector<QVector3D> m_surfaceNormals;
    QVector<GLint>     m_surfaceFace;
	std::vector<double> m_surfaceArea;
	std::vector<int>	m_surfaceVertMask;
    
    QVector<QVector3D> m_innerVertices;
    QVector<QVector3D> m_innerNormals;
    QVector<GLint>     m_innerFace;
   
    int m_scaleLevel;
    const double *m_vertForce;
    float m_forceScale;
    int m_numVert;
    QVector<QVector3D> m_forceBuffer;

    std::vector<int> m_selectedNode;
};

