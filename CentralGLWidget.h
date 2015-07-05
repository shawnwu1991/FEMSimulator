#pragma once

#include <QGLWidget>
#include <QtCore/QVector>
#include <QtOpenGL/QGLShaderProgram>
#include <QTime>
#include <QVector>
#include <QtCore\QEvent>

#include "comHeader.h"
#include "GLTetMesh.h"
#include "Camera.h"


class CentralGLWidget : public QGLWidget
{
    Q_OBJECT

public:
    CentralGLWidget(QWidget *parent = 0);
    ~CentralGLWidget(void);

signals:
    void Draw(void);

public slots:
    void OnSetForceScaleLevel(int scalelevel);


public:
    void PushModelMatrix(void);
    void PopModelMatrix(void);

    static QString LoadStringFromFile(const QString &filename);

    int AddTetMesh(QSharedPointer<GLTetMesh> tetMesh);
    void UpdateTetMesh(int id = -1);
    QSharedPointer<GLTetMesh> GetTetMesh(int id) { return m_GLTetMesh[id]; }
    int GetTetMeshNum() { return m_GLTetMesh.size(); }
	void ClearTetMesh(void);

    QMatrix4x4 &ModelMatrix(void) { return m_modelMatrix; }

    void DrawBox(void);
    void DrawTextLine(const QString &str) { m_lineToPrint = str; }

	void DumpImage();
	bool SaveCamera(const tstring &vertFile);
	bool LoadCamera(const tstring &vertFile);
	void setWindInitialPos(double x, double y, double z);
	void setWindVelocity(std::vector<Vector3d> vel);
	void setWindForce(std::vector<Vector3d> f);
	void setWindRange(int range);
	void setContactForce(std::vector<Vector3d> f);
	void setDrawWindVelocity(bool b) {m_drawWindVelocity = b;}
	void setDrawWindForce(bool b) {m_drawWindForce=b;}
	void setDrawContactForce(bool b) {m_drawContactForce = b;}
	Vector3d &getWindFirstNode() {return m_windFirstNodePos;}
	int &getWindRange() {return m_windRange;}

public:
    void SetupProjViewModelMatrixForShader(QGLShaderProgram &shader);
    void SetupLightsForShader(QGLShaderProgram &shader);

protected:
    virtual void paintGL(void);
    virtual void initializeGL(void);
    virtual void resizeGL(int width, int height);


    virtual void mousePressEvent(QMouseEvent * event);
    virtual void mouseReleaseEvent(QMouseEvent * event);
    virtual void mouseMoveEvent(QMouseEvent * event);
    virtual void mouseDoubleClickEvent(QMouseEvent * event);

private:
    // ground line
    void InitializeGroundLines(void);
    void DrawGroundLine(void);

    // default shader
    void InitializeDefaultShader(void);

    // draw wind
	void DrawWind(void);
	void DrawWindForce(void);
	void DrawContactForce(void);

private:
    // camera
    Camera m_camera;

    // mouse
    Qt::MouseButton m_pressedBtn; 
    QPoint m_oldMousePos;

    // matrices
    QMatrix4x4 m_modelMatrix;
    QStack<QMatrix4x4> m_modelMatrixStack;

    // colors
    QVector4D m_clearColor;   

    QString m_lineToPrint;

    // ground line
    bool m_flagDrawGroundLine;
    QVector4D m_groundLineColor;
    QVector<QVector3D> m_groundLines;
    QGLShaderProgram m_groundShader;
    struct GroundLineShaderVarLocation
    {
        // attrib
        int vertex;
        // uniform
        int projViewModelmatrix;
        int color;
    } m_groundShaderVars;

    // default shader
    QGLShaderProgram m_defaultShader;
    struct DefaultShaderVarLocation
    {
        // attrib
        int aVertex;
        int aNormal;

        // uniform
        int uProjViewModelmatrix;
        int uViewModelMatrix;
        int uProjMatrix;
        int uViewMatrix;
        int uModelMatrix;        
        int uNormalMatrix;        
        int uLightPos;

        struct LightMaterial
        {
            int ambient;
            int diffuse;
            int specular;
        } uLightMaterial;

        struct ObjectMaterial
        {
            int diffuse;
            int specular;
            int shininess;
        } uObjectMaterial;
    } m_defaultShaderVars;
    
    // TetMesh
    void DrawTetMesh(void);
    //QVector<QSharedPointer<GLTetMesh>> m_tetMesh;
	QVector<QSharedPointer<GLTetMesh>> m_GLTetMesh;

	// Dump
	QImage      m_imageShot;

    int m_scaleLevel;

	//wind
	int m_windRange;
	Vector3d m_windFirstNodePos;
	bool m_drawWindVelocity;
	bool m_drawWindForce;
	std::vector<Vector3d> m_glWindVelocity;
	QVector<QVector3D> m_glWindVelBuffer;
	QGLShaderProgram m_windVelShader;

	std::vector<Vector3d> m_glWindForce;
	QVector<QVector3D> m_glWindForceBuffer;
	QGLShaderProgram m_windForceShader;

	//Contact
	bool m_drawContactForce;
	std::vector<Vector3d> m_glContactForce;
	QVector<QVector3D> m_glContactForceBuffer;
	QGLShaderProgram m_contactForceShader;

};

