#include "stdafx.h"
#include "CentralGLWidget.h"

#include <QPainter>
#include <QPaintEngine>
#include <QSurface>
#include <math.h>
#include <string>
#include <fstream>


CentralGLWidget::CentralGLWidget(QWidget *parent)
	: QGLWidget(parent)
	, m_clearColor(1.0f, 1.0f, 1.0f, 1.0f)
	, m_groundLineColor(0.4f, 0.4f, 0.4f, 1.0f)
	, m_flagDrawGroundLine(true)
	, m_pressedBtn(Qt::MouseButton::NoButton)
	, m_scaleLevel(0)
	, m_drawWindVelocity(true)
	, m_drawWindForce(true)
	, m_drawContactForce(true)
{
	setAttribute(Qt::WA_PaintOnScreen);
	setAttribute(Qt::WA_NoSystemBackground);
	setAutoBufferSwap(false);
	setMinimumSize(300, 300);

	m_modelMatrix.setToIdentity();
}

CentralGLWidget::~CentralGLWidget(void)
{
}

QString CentralGLWidget::LoadStringFromFile(const QString &filename)
{
	QFile file(filename);    
	if (!file.open(QIODevice::ReadOnly))
		printf("Can't open file %s\n", qPrintable(filename));

	return QTextStream(&file).readAll();
}

void CentralGLWidget::PushModelMatrix(void)
{
	m_modelMatrixStack.push(m_modelMatrix);
}

void CentralGLWidget::PopModelMatrix(void)
{
	m_modelMatrix = m_modelMatrixStack.pop();
}

void CentralGLWidget::initializeGL(void)
{
	// ground 
	InitializeGroundLines();

	// default shader
	InitializeDefaultShader();

	//m_windVelShader.addShaderFromSourceFile(QGLShader::Vertex, ResourcePath(QStringLiteral("Shaders/TetForce.vert")));
	//m_windVelShader.addShaderFromSourceFile(QGLShader::Fragment, ResourcePath(QStringLiteral("Shaders/TetForce.frag")));
	//m_windVelShader.link();

	//m_windForceShader.addShaderFromSourceFile(QGLShader::Vertex, ResourcePath(QStringLiteral("Shaders/TetForce.vert")));
	//m_windForceShader.addShaderFromSourceFile(QGLShader::Fragment, ResourcePath(QStringLiteral("Shaders/TetForce.frag")));
	//m_windForceShader.link();

	m_contactForceShader.addShaderFromSourceFile(QGLShader::Vertex, ResourcePath(QStringLiteral("Shaders/TetForce.vert")));
	m_contactForceShader.addShaderFromSourceFile(QGLShader::Fragment, ResourcePath(QStringLiteral("Shaders/TetForce.frag")));
	m_contactForceShader.link();
}

void CentralGLWidget::resizeGL(int width, int height)
{
	m_camera.SetAspectRatio((float)width, (float)height);
}

void CentralGLWidget::paintGL(void)
{
	QPainter painter;

	painter.begin(this);
	painter.setRenderHint(QPainter::Antialiasing);
	painter.setRenderHint(QPainter::HighQualityAntialiasing);

	painter.beginNativePainting();

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClear(GL_DEPTH_BUFFER_BIT);

	glClearColor(m_clearColor.x(), m_clearColor.y(), m_clearColor.z(), m_clearColor.w());
	glClear(GL_COLOR_BUFFER_BIT);

	// default gl settings
	//glFrontFace(GL_CCW);
	//glEnable(GL_CULL_FACE);
	//glCullFace(GL_BACK);

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	m_modelMatrixStack.clear();

	// draw something
	DrawGroundLine();
	//DrawBoxSphere();
	DrawTetMesh();
	glDepthMask(false);
	//DrawWind();
	//DrawWindForce();
	DrawContactForce();
	//glDisable(GL_CULL_FACE);

	glDisable(GL_DEPTH_TEST);

	painter.endNativePainting();

	if (!m_lineToPrint.isEmpty())
	{
		painter.drawText(20, 20, m_lineToPrint);
	}

	painter.end();

	swapBuffers();
}

void CentralGLWidget::InitializeGroundLines(void)
{
	// prepare line vertices
	m_groundLines.clear();
	float scale = 0.2f;

	m_groundLines << QVector3D( 0.0f, 0.0f, 10.0f * scale) << QVector3D(0.0f, 0.0f, -10.0f * scale);
	m_groundLines << QVector3D( 10.0f * scale, 0.0f, 0.0f) << QVector3D(-10.0f * scale, 0.0f, 0.0f);
	m_groundLines << QVector3D( 0.01f, 0.0f, 10.0f * scale) << QVector3D(0.01f, 0.0f, -10.0f * scale);
	m_groundLines << QVector3D( 10.0f * scale, 0.0f, 0.01f) << QVector3D(-10.0f * scale, 0.0f, 0.01f);
	m_groundLines << QVector3D(-0.01f, 0.0f, 10.0f * scale) << QVector3D(-0.01f, 0.0f, -10.0f * scale);
	m_groundLines << QVector3D( 10.0f * scale, 0.0f, -0.01f) << QVector3D(-10.0f * scale, 0.0f, -0.01f);
	for (float i = -10.0f; i < 0.0f; ++i)
	{
		m_groundLines << QVector3D( i * scale, 0.0f, 10.0f * scale) << QVector3D(i * scale, 0.0f, -10.0f * scale);
		m_groundLines << QVector3D( 10.0f * scale, 0.0f, i * scale) << QVector3D(-10.0f * scale, 0.0f, i * scale);
	}
	for (float i = 1.0f; i <= 10.0f; ++i)
	{
		m_groundLines << QVector3D( i * scale, 0.0f, 10.0f * scale) << QVector3D(i * scale, 0.0f, -10.0f * scale);
		m_groundLines << QVector3D( 10.0f * scale, 0.0f, i * scale) << QVector3D(-10.0f * scale, 0.0f, i * scale);
	}

	// prepare shader
	//QGLShader *vshader = new QGLShader(QGLShader::Vertex, this);
	////QString vshaderStr = LoadStringFromFile(QStringLiteral(":/FEMSimulator/Resources/Ground.vert"));
	//QString vshaderStr = LoadStringFromFile(QStringLiteral("Resources/Ground.vert"));
	//vshader->compileSourceCode(vshaderStr);

	//QGLShader *fshader = new QGLShader(QGLShader::Fragment, this);
	////QString fshaderStr = LoadStringFromFile(QStringLiteral(":/FEMSimulator/Resources/Ground.frag"));
	//QString fshaderStr = LoadStringFromFile(QStringLiteral("Resources/Ground.frag"));
	//fshader->compileSourceCode(fshaderStr);

	if (!m_groundShader.addShaderFromSourceFile(QGLShader::Vertex, ResourcePath(QStringLiteral("Shaders/Ground.vert")))){
		QMessageBox::warning(this, "QGLShader::Vertex", "QGLShader::Vertex " + m_groundShader.log());
	}

	// Compile fragment shader
	if (!m_groundShader.addShaderFromSourceFile(QGLShader::Fragment, ResourcePath(QStringLiteral("Shaders/Ground.frag")))){
		QMessageBox::warning(this, "QGLShader::Fragment", "QGLShader::Fragment " + m_groundShader.log());
	}

	//m_groundShader.addShader(vshader);
	//m_groundShader.addShader(fshader);
	m_groundShader.link();

	m_groundShaderVars.vertex = m_groundShader.attributeLocation("vertex");

	m_groundShaderVars.projViewModelmatrix = m_groundShader.uniformLocation("projViewModelmatrix");
	m_groundShaderVars.color = m_groundShader.uniformLocation("color");

	m_groundShader.setUniformValue(m_groundShaderVars.color, m_groundLineColor);
}

void CentralGLWidget::DrawGroundLine(void)
{     
	m_groundShader.bind();
	// model matrix is always identity
	m_groundShader.setUniformValue(m_groundShaderVars.projViewModelmatrix, m_camera.GetProjViewMatrix());  
	m_groundShader.setUniformValue(m_groundShaderVars.color, m_groundLineColor);

	m_groundShader.enableAttributeArray(m_groundShaderVars.vertex);

	m_groundShader.setAttributeArray(m_groundShaderVars.vertex, m_groundLines.constData());
	glDrawArrays(GL_LINES, 0, m_groundLines.size());

	m_groundShader.disableAttributeArray(m_groundShaderVars.vertex);
	m_groundShader.release();

}


void CentralGLWidget::mousePressEvent(QMouseEvent * event)
{
	m_pressedBtn = event->button();
	m_camera.OnMousePressEvent(QVector2D((float)event->x()/(float)width(), 
		1.0f - (float)event->y()/(float)height()));

	m_oldMousePos = event->pos();

	event->accept();
}

void CentralGLWidget::mouseReleaseEvent(QMouseEvent * event)
{
	m_pressedBtn = Qt::MouseButton::NoButton;
	m_camera.OnMouseReleaseEvent(QVector2D((float)event->x()/(float)width(), 
		1.0f - (float)event->y()/(float)height()));

	m_oldMousePos = event->pos();

	event->accept();
}

void CentralGLWidget::mouseMoveEvent(QMouseEvent * event)
{
	if (m_pressedBtn == Qt::MouseButton::NoButton)
		return;

	QVector2D pt((float)event->x()/(float)width(), 
		1.0f - (float)event->y()/(float)height());
	QVector2D oldpt((float)m_oldMousePos.x()/(float)width(), 
		1.0f - (float)m_oldMousePos.y()/(float)height());
	switch (m_pressedBtn)
	{
	case Qt::MouseButton::LeftButton:        
		m_camera.OnMouseMoveEvent(pt, oldpt, Camera::DragType::Drag_Rotate);
		break;

	case Qt::MouseButton::RightButton:
		m_camera.OnMouseMoveEvent(pt, oldpt, Camera::DragType::Drag_Zoom);
		break;

	case Qt::MouseButton::MidButton:
		m_camera.OnMouseMoveEvent(pt, oldpt, Camera::DragType::Drag_Move);
		break;
	}

	m_oldMousePos = event->pos();

	update();
}

void CentralGLWidget::mouseDoubleClickEvent(QMouseEvent * event)
{
	if (event->modifiers() & Qt::ControlModifier)
	{
		m_camera.ResetView();
		event->accept();
		update();
	}
}

void CentralGLWidget::InitializeDefaultShader(void)
{
	m_defaultShader.addShaderFromSourceFile(QGLShader::Vertex, ResourcePath(QStringLiteral("Shaders/Default.vert")));
	m_defaultShader.addShaderFromSourceFile(QGLShader::Fragment, ResourcePath(QStringLiteral("Shaders/Default.frag")));

	m_defaultShader.link();

	// var location
	m_defaultShaderVars.aVertex                   = m_defaultShader.attributeLocation("aVertex");
	m_defaultShaderVars.aNormal                   = m_defaultShader.attributeLocation("aNormal");

	m_defaultShaderVars.uProjViewModelmatrix      = m_defaultShader.uniformLocation("uProjViewModelmatrix");
	m_defaultShaderVars.uViewModelMatrix          = m_defaultShader.uniformLocation("uViewModelMatrix");
	m_defaultShaderVars.uProjMatrix               = m_defaultShader.uniformLocation("uProjMatrix");
	m_defaultShaderVars.uViewMatrix               = m_defaultShader.uniformLocation("uViewMatrix");
	m_defaultShaderVars.uModelMatrix              = m_defaultShader.uniformLocation("uModelMatrix");        
	m_defaultShaderVars.uNormalMatrix             = m_defaultShader.uniformLocation("uNormalMatrix");        
	m_defaultShaderVars.uLightPos                 = m_defaultShader.uniformLocation("uLightPos");
	m_defaultShaderVars.uLightMaterial.ambient    = m_defaultShader.uniformLocation("uLightMaterial.ambient");
	m_defaultShaderVars.uLightMaterial.diffuse    = m_defaultShader.uniformLocation("uLightMaterial.diffuse");
	m_defaultShaderVars.uLightMaterial.specular   = m_defaultShader.uniformLocation("uLightMaterial.specular");
	m_defaultShaderVars.uObjectMaterial.diffuse   = m_defaultShader.uniformLocation("uObjectMaterial.diffuse");
	m_defaultShaderVars.uObjectMaterial.specular  = m_defaultShader.uniformLocation("uObjectMaterial.specular");
	m_defaultShaderVars.uObjectMaterial.shininess = m_defaultShader.uniformLocation("uObjectMaterial.shininess");
}

void CentralGLWidget::DrawBox(void)
{
#pragma region BoxDef
	QVector3D bv[] = {
		QVector3D( 0.5, -0.5, 0.5),
		QVector3D( 0.5,  0.5, 0.5),
		QVector3D(-0.5,  0.5, 0.5),
		QVector3D(-0.5, -0.5, 0.5),

		QVector3D( 0.5, -0.5, -0.5),
		QVector3D( 0.5,  0.5, -0.5),
		QVector3D(-0.5,  0.5, -0.5),
		QVector3D(-0.5, -0.5, -0.5),
	};
	int boxid[] = {
		0, 1, 2,    0, 2, 3, //0, 1, 2, 3,
		0, 4, 5,    0, 5, 1, //0, 4, 5, 1,
		1, 5, 6,    1, 6, 2, //1, 5, 6, 2,
		2, 6, 7,    2, 7, 3, //2, 6, 7, 3,
		3, 7, 4,    3, 4, 0, //3, 7, 4, 0,
		4, 7, 6,    4, 6, 5  //4, 7, 6, 5
	};
	QVector<QVector3D> box;
	for (auto bi : boxid)
		box << bv[bi];

	QVector2D bt[] = {
		QVector2D(0.0, 0.0),  QVector2D(0.0, 1.0),  QVector2D(1.0, 1.0),  
		QVector2D(0.0, 0.0),  QVector2D(1.0, 1.0),  QVector2D(1.0, 0.0),

		QVector2D(0.0, 0.0),  QVector2D(0.0, 1.0),  QVector2D(1.0, 1.0), 
		QVector2D(0.0, 0.0),  QVector2D(1.0, 1.0),  QVector2D(1.0, 0.0),

		QVector2D(0.0, 0.0),  QVector2D(0.0, 1.0),  QVector2D(1.0, 1.0),  
		QVector2D(0.0, 0.0),  QVector2D(1.0, 1.0),  QVector2D(1.0, 0.0),

		QVector2D(0.0, 0.0),  QVector2D(0.0, 1.0),  QVector2D(1.0, 1.0),  
		QVector2D(0.0, 0.0),  QVector2D(1.0, 1.0),  QVector2D(1.0, 0.0),

		QVector2D(0.0, 0.0),  QVector2D(0.0, 1.0),  QVector2D(1.0, 1.0), 
		QVector2D(0.0, 0.0),  QVector2D(1.0, 1.0),  QVector2D(1.0, 0.0),

		QVector2D(0.0, 0.0),  QVector2D(0.0, 1.0),  QVector2D(1.0, 1.0),  
		QVector2D(0.0, 0.0),  QVector2D(1.0, 1.0),  QVector2D(1.0, 0.0)
	};

	QVector3D bn[] = {
		QVector3D( 0.0,  0.0,  1.0),  QVector3D( 0.0,  0.0,  1.0),  QVector3D( 0.0,  0.0,  1.0),
		QVector3D( 0.0,  0.0,  1.0),  QVector3D( 0.0,  0.0,  1.0),  QVector3D( 0.0,  0.0,  1.0),

		QVector3D( 1.0,  0.0,  0.0),  QVector3D( 1.0,  0.0,  0.0),  QVector3D( 1.0,  0.0,  0.0),
		QVector3D( 1.0,  0.0,  0.0),  QVector3D( 1.0,  0.0,  0.0),  QVector3D( 1.0,  0.0,  0.0),

		QVector3D( 0.0,  1.0,  0.0),  QVector3D( 0.0,  1.0,  0.0),  QVector3D( 0.0,  1.0,  0.0),
		QVector3D( 0.0,  1.0,  0.0),  QVector3D( 0.0,  1.0,  0.0),  QVector3D( 0.0,  1.0,  0.0),

		QVector3D(-1.0,  0.0,  0.0),  QVector3D(-1.0,  0.0,  0.0),  QVector3D(-1.0,  0.0,  0.0),
		QVector3D(-1.0,  0.0,  0.0),  QVector3D(-1.0,  0.0,  0.0),  QVector3D(-1.0,  0.0,  0.0),

		QVector3D( 0.0, -1.0,  0.0),  QVector3D( 0.0, -1.0,  0.0),  QVector3D( 0.0, -1.0,  0.0),
		QVector3D( 0.0, -1.0,  0.0),  QVector3D( 0.0, -1.0,  0.0),  QVector3D( 0.0, -1.0,  0.0),

		QVector3D( 0.0,  0.0, -1.0),  QVector3D( 0.0,  0.0, -1.0),  QVector3D( 0.0,  0.0, -1.0),
		QVector3D( 0.0,  0.0, -1.0),  QVector3D( 0.0,  0.0, -1.0),  QVector3D( 0.0,  0.0, -1.0)
	};
#pragma endregion BoxDef

	auto &shader = m_defaultShader;
	auto &var = m_defaultShaderVars;

	shader.bind();

	auto projViewModelMat = m_camera.GetProjViewMatrix() * m_modelMatrix;
	shader.setUniformValue(var.uProjViewModelmatrix, projViewModelMat);
	shader.setUniformValue(var.uViewModelMatrix    , m_camera.GetViewMatrix() * m_modelMatrix);
	shader.setUniformValue(var.uProjMatrix         , m_camera.GetProjectMatrix());
	shader.setUniformValue(var.uViewMatrix         , m_camera.GetViewMatrix());
	shader.setUniformValue(var.uModelMatrix        , m_modelMatrix);
	shader.setUniformValue(var.uNormalMatrix       , projViewModelMat.normalMatrix());
	shader.setUniformValue(var.uLightPos           , QVector4D(10.0f, 15.0f, 20.0f, 1.0f));

	shader.setUniformValue(m_defaultShaderVars.uLightMaterial.ambient   , QVector4D(0.35f, 0.35f, 0.35f, 1.0f));
	shader.setUniformValue(m_defaultShaderVars.uLightMaterial.diffuse   , QVector4D(0.5f, 0.5f, 0.5f, 1.0f));
	shader.setUniformValue(m_defaultShaderVars.uLightMaterial.specular  , QVector4D(0.8f, 0.8f, 0.8f, 1.0f));

	shader.setUniformValue(m_defaultShaderVars.uObjectMaterial.diffuse  , QVector4D(0.906f, 0.612f, 0.196f, 1.0f));
	shader.setUniformValue(m_defaultShaderVars.uObjectMaterial.specular , QVector4D(0.5f, 0.5f, 0.5f, 1.0f));
	shader.setUniformValue(m_defaultShaderVars.uObjectMaterial.shininess, 10.0f);

	shader.enableAttributeArray(var.aVertex);
	shader.enableAttributeArray(var.aNormal);
	shader.setAttributeArray(var.aVertex, box.constData());
	shader.setAttributeArray(var.aNormal, bn);
	glDrawArrays(GL_TRIANGLES, 0, box.size());
	shader.disableAttributeArray(var.aVertex);
	shader.disableAttributeArray(var.aNormal);
	shader.release();
}

int CentralGLWidget::AddTetMesh(QSharedPointer<GLTetMesh> tetMesh)
{
	m_GLTetMesh.push_back(tetMesh);
	tetMesh->OnSetForceScaleLevel(m_scaleLevel);
	return (int) m_GLTetMesh.size() - 1;
}

void CentralGLWidget:: UpdateTetMesh(int id /*= -1*/)
{
	if (id < 0)
	{
		for (auto p : m_GLTetMesh)
			p->UpdateVertexNormal();
	}
	else if (id < (int) m_GLTetMesh.size())
		m_GLTetMesh[id]->UpdateVertexNormal();
}

void CentralGLWidget::ClearTetMesh(void)
{
	m_GLTetMesh.clear();
}


void CentralGLWidget::DrawTetMesh(void)
{
	for (auto tetMesh : m_GLTetMesh)
	{
		tetMesh->DrawTetMesh();
	}
}

void CentralGLWidget::SetupProjViewModelMatrixForShader(QGLShaderProgram &shader)
{    
	auto projViewModelMat = m_camera.GetProjViewMatrix() * m_modelMatrix;
	shader.setUniformValue("uProjViewModelmatrix", projViewModelMat);
	shader.setUniformValue("uViewModelMatrix"    , m_camera.GetViewMatrix() * m_modelMatrix);
	shader.setUniformValue("uProjMatrix"         , m_camera.GetProjectMatrix());
	shader.setUniformValue("uViewMatrix"         , m_camera.GetViewMatrix());
	shader.setUniformValue("uModelMatrix"        , m_modelMatrix);
	shader.setUniformValue("uNormalMatrix"       , projViewModelMat.normalMatrix());
}

void CentralGLWidget::SetupLightsForShader(QGLShaderProgram &shader)
{
	shader.setUniformValue("uLightPos"                , QVector4D(10.0f, 15.0f, 20.0f, 1.0f));
	shader.setUniformValue("uLightMaterial.ambient"   , QVector4D(0.35f, 0.35f, 0.35f, 1.0f));
	shader.setUniformValue("uLightMaterial.diffuse"   , QVector4D(0.5f, 0.5f, 0.5f, 1.0f));
	shader.setUniformValue("uLightMaterial.specular"  , QVector4D(0.8f, 0.8f, 0.8f, 1.0f));
}

void CentralGLWidget::OnSetForceScaleLevel(int scalelevel)
{
	m_scaleLevel = scalelevel;
	for (auto tetMesh : m_GLTetMesh)
		tetMesh->OnSetForceScaleLevel(scalelevel);

	update();
}


void CentralGLWidget::DumpImage()
{
	update();

	static int index = 1;
	QString format = "bmp";
	QString fileName = QString::number(index);
	fileName = "Capture/"+fileName + "." + format;
	QString filePath = QDir::toNativeSeparators(fileName);
	m_imageShot = this->grabFrameBuffer();
	m_imageShot.save(filePath, format.toLatin1());
	index++;
}

bool CentralGLWidget::SaveCamera(const tstring &vertFile)
{
	tofstream fout(vertFile);
	if (!fout.is_open())
	{
		std::tcout << TEXT("Can't open ") << vertFile << std::endl;
		return false;
	}

	fout.precision(18);
	fout << TEXT("Eye ") << m_camera.GetCameraEye().x() << TEXT(" ") << m_camera.GetCameraEye().y() << TEXT(" ") << m_camera.GetCameraEye().z() << std::endl;
	fout << TEXT("Center ") << m_camera.GetCameraCenter().x() << TEXT(" ") << m_camera.GetCameraCenter().y() << TEXT(" ") << m_camera.GetCameraCenter().z() << std::endl;
	fout << TEXT("Up ") << m_camera.GetCameraUp().x() << TEXT(" ") << m_camera.GetCameraUp().y() << TEXT(" ") << m_camera.GetCameraUp().z() << std::endl;

	fout << TEXT("fov ") << m_camera.GetCameraFov() << std::endl;
	fout << TEXT("aspect ") << m_camera.GetCameraAspect() << std::endl;
	fout << TEXT("near ") << m_camera.GetCameraNearPlane() << std::endl;
	fout << TEXT("far ") << m_camera.GetCameraFarPlane() << std::endl;

	return true;
}

bool CentralGLWidget::LoadCamera(const tstring &camFile)
{
	tifstream fin(camFile);
	if (!fin.is_open())
	{
		std::tcout << TEXT("Can't open ") << camFile << std::endl;
		return false;
	}

	tstring line;
	tstring prefix;
	tistringstream sstr;

	// eye
	QVector3D eye;
	std::getline(fin, line);
	sstr.clear();
	sstr.str(line.c_str());
	sstr >> prefix >> eye[0] >> eye[1] >> eye[2];

	// center
	QVector3D center;
	std::getline(fin, line);
	sstr.clear();
	sstr.str(line.c_str());
	sstr >> prefix >> center[0] >> center[1] >> center[2];

	// up
	QVector3D up;
	std::getline(fin, line);
	sstr.clear();
	sstr.str(line.c_str());
	sstr >> prefix >> up[0] >> up[1] >> up[2];

	// fob
	float fov;
	std::getline(fin, line);
	sstr.clear();
	sstr.str(line.c_str());
	sstr >> prefix >> fov;

	// center
	float aspect;
	std::getline(fin, line);
	sstr.clear();
	sstr.str(line.c_str());
	sstr >> prefix >> aspect;

	// near
	float nearPlane;
	std::getline(fin, line);
	sstr.clear();
	sstr.str(line.c_str());
	sstr >> prefix >> nearPlane;

	// far
	float farPlane;
	std::getline(fin, line);
	sstr.clear();
	sstr.str(line.c_str());
	sstr >> prefix >> farPlane;

	m_camera.LookAt(eye, center, up);
	m_camera.Perspective(fov, aspect, nearPlane, farPlane);


	return true;
}

void CentralGLWidget::setWindInitialPos(double x, double y, double z)
{
	m_windFirstNodePos[0] = x - 15 * 0.05;
	m_windFirstNodePos[1] = y - 15 * 0.05;
	m_windFirstNodePos[2] = z - 15 * 0.05;
}

void CentralGLWidget::setWindVelocity(std::vector<Vector3d> vel)
{
	if (vel.size() != 0)
	{
		m_glWindVelocity = vel;
		m_glWindVelBuffer.resize(vel.size()*2);
	}
	else
	{
		m_glWindVelocity.clear();
		m_glWindVelBuffer.clear();
	}
	this->updateGL();
}

void CentralGLWidget::setWindForce(std::vector<Vector3d> f)
{
	if (f.size() != 0)
	{
		m_glWindForce = f;
		m_glWindForceBuffer.resize(f.size()*2);
	}
	else
	{
		m_glWindForce.clear();
		m_glWindForceBuffer.clear();
	}
	this->updateGL();
}

void CentralGLWidget::setContactForce(std::vector<Vector3d> f)
{
		if (f.size() != 0)
	{
		m_glContactForce = f;
		m_glContactForceBuffer.resize(f.size()*2);
	}
	else
	{
		m_glContactForce.clear();
		m_glContactForceBuffer.clear();
	}
	this->updateGL();
}

void CentralGLWidget::setWindRange(int range)
{
	m_windRange = range;
}

void CentralGLWidget::DrawWind(void)
{
	if (m_drawWindVelocity && m_glWindVelocity.size() != 0 )
	{
		int rangesquare = m_windRange * m_windRange;
		for (int i=0;i<m_windRange;++i)
		{
			for (int j=0;j<m_windRange;++j)
			{
				for (int k=0;k<m_windRange;++k)
				{
					m_glWindVelBuffer[(rangesquare*i + m_windRange*j + k)*2] = 
						QVector3D(m_windFirstNodePos.X()+i*0.05,
							      m_windFirstNodePos.Y()+j*0.05,
								  m_windFirstNodePos.Z()+k*0.05);
	
					m_glWindVelBuffer[(rangesquare*i + m_windRange*j + k)*2+1] = 
					m_glWindVelBuffer[(rangesquare*i + m_windRange*j + k)*2] +
			          0.02 * QVector3D(m_glWindVelocity[rangesquare*i + m_windRange*j + k].X(),
									   m_glWindVelocity[rangesquare*i + m_windRange*j + k].Y(),
							           m_glWindVelocity[rangesquare*i + m_windRange*j + k].Z());
				}
			}
		}
	
		m_windVelShader.bind();
		SetupProjViewModelMatrixForShader(m_windVelShader);
		m_windVelShader.setUniformValue("color",QVector4D(0.8,0.6,0.6,0.1));
		m_windVelShader.enableAttributeArray("aVertex");
		m_windVelShader.setAttributeArray("aVertex", m_glWindVelBuffer.constData());
		glDrawArrays(GL_LINES,0,m_glWindVelBuffer.size());
		m_windVelShader.disableAttributeArray("aVertex");
		m_windVelShader.release();
	}
}

void CentralGLWidget::DrawWindForce(void)
{
	if ( m_drawWindForce && m_glWindForce.size() != 0)
	{
		for (int i=0; i<m_glWindForce.size();++i)
		{
			m_glWindForceBuffer[i*2] = QVector3D(m_GLTetMesh[0]->GetTetmesh()->GetVertices()[i].X(),
												 m_GLTetMesh[0]->GetTetmesh()->GetVertices()[i].Y(),
												 m_GLTetMesh[0]->GetTetmesh()->GetVertices()[i].Z());
			m_glWindForceBuffer[i*2+1] = m_glWindForceBuffer[i*2] +
									     5*QVector3D(m_glWindForce[i].X(),m_glWindForce[i].Y(),m_glWindForce[i].Z());
		}

		m_windForceShader.bind();
		SetupProjViewModelMatrixForShader(m_windForceShader);
		m_windForceShader.setUniformValue("color", QVector4D(0.1,0.1,0.9,0.1));
		m_windForceShader.enableAttributeArray("aVertex");
		m_windForceShader.setAttributeArray("aVertex",m_glWindForceBuffer.constData());
		glDrawArrays(GL_LINES,0,m_glWindForceBuffer.size());
		m_windForceShader.disableAttributeArray("aVertex");
		m_windForceShader.release();
	}
}

void CentralGLWidget::DrawContactForce(void)
{
	if ( m_drawContactForce && m_glContactForce.size() != 0)
	{
		for (int i=0; i<m_glContactForce.size();++i)
		{
			m_glContactForceBuffer[i*2] = QVector3D(m_GLTetMesh[0]->GetTetmesh()->GetVertices()[i].X(),
													m_GLTetMesh[0]->GetTetmesh()->GetVertices()[i].Y(),
													m_GLTetMesh[0]->GetTetmesh()->GetVertices()[i].Z());
			m_glContactForceBuffer[i*2+1] = m_glContactForceBuffer[i*2] +
									     5*QVector3D(m_glContactForce[i].X(),m_glContactForce[i].Y(),m_glContactForce[i].Z());
		}

		m_contactForceShader.bind();
		SetupProjViewModelMatrixForShader(m_contactForceShader);
		m_contactForceShader.setUniformValue("color", QVector4D(0.1,0.1,0.9,0.1));
		m_contactForceShader.enableAttributeArray("aVertex");
		m_contactForceShader.setAttributeArray("aVertex",m_glContactForceBuffer.constData());
		glDrawArrays(GL_LINES,0,m_glContactForceBuffer.size());
		m_contactForceShader.disableAttributeArray("aVertex");
		m_contactForceShader.release();
	}
}



