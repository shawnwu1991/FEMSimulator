#include "stdafx.h"
#include "GLTetMesh.h"
#include "CentralGLWidget.h"

#include "InlinedShaderCode.h"

#include <vector>
#include <map>
#include <array>

GLTetMesh::GLTetMesh(CentralGLWidget *glWidget, std::shared_ptr<Libin::TetMesh> tetMesh)
    : m_glWidget(glWidget)
    , m_tetMesh(tetMesh)
    , m_vertForce(NULL)
    , m_numVert(0)
    , m_forceScale(0)
    , m_scaleLevel(0)
{
    InitializeShaders();
    Initialize();
	m_tetMesh->SetSurfaceFaces(m_surfaceFace);
}


GLTetMesh::~GLTetMesh(void)
{
}

void GLTetMesh::InitializeShaders(void)
{
    m_surfaceShader.addShaderFromSourceFile(QGLShader::Vertex, ResourcePath(QStringLiteral("Shaders/Default.vert")));
    m_surfaceShader.addShaderFromSourceFile(QGLShader::Fragment, ResourcePath(QStringLiteral("Shaders/Default.frag")));
    m_surfaceShader.link();
    
    m_forceShader.addShaderFromSourceFile(QGLShader::Vertex, ResourcePath(QStringLiteral("Shaders/TetForce.vert")));
    m_forceShader.addShaderFromSourceFile(QGLShader::Fragment, ResourcePath(QStringLiteral("Shaders/TetForce.frag")));
    m_forceShader.link();
}

void GLTetMesh::Initialize(void)
{
    if (!m_tetMesh)
    {
        printf("Error! NULL TetMesh pointer!\n");
        return;
    }

    double *vertices = reinterpret_cast<double *>(m_tetMesh->GetVertices().data());
    int nElement = m_tetMesh->GetTopology()->GetNumElement();
    auto &elements = m_tetMesh->GetTopology()->GetElements();
    
    std::map<std::array<int, 3>, std::pair<int, int>> facesTypeElm;
    for (int i = 0; i < nElement; ++i)
    {
        auto &elmId = elements[i];

        std::array<int, 3> faces[] = {
            { elmId[0] , elmId[2], elmId[1] },
            { elmId[0] , elmId[1], elmId[3] },
            { elmId[0] , elmId[3], elmId[2] },
            { elmId[1] , elmId[2], elmId[3] }
        };

        for (auto &f : faces)
        {
            std::sort(f.begin(), f.end());
            auto it = facesTypeElm.find(f);
            if (it == facesTypeElm.end())
                facesTypeElm[f] = std::pair<int, int>(0, i);
            else
                ++it->second.first;
        }
    }

    m_surfaceVertices.clear();
    m_surfaceNormals.clear();
    m_surfaceFace.clear();    
    m_innerVertices.clear();
    m_innerNormals.clear();
    m_innerFace.clear();
    
    // saparate surface/inner faces
    for (auto &faceInfo : facesTypeElm)
    {
        if (faceInfo.second.first == 0)    // surface face
        {
            // reorder the vertices to make outward normals
            auto elmId = elements[faceInfo.second.second];
            std::sort(elmId.begin(), elmId.end());
            int refId[1];
            std::set_difference(elmId.begin(), elmId.end(), faceInfo.first.begin(), faceInfo.first.end(), refId);
            QVector3D v[3];
            for (int i = 0; i < 3; ++i)
            {
                v[i] = QVector3D((float) vertices[faceInfo.first[i] * 3], 
                    (float) vertices[faceInfo.first[i] * 3 + 1], (float) vertices[faceInfo.first[i] * 3 + 2]);
            }
            QVector3D vref((float) vertices[*refId * 3], (float) vertices[*refId * 3 + 1], (float) vertices[*refId * 3 + 2]);
            if (QVector3D::dotProduct(QVector3D::crossProduct(v[1] - v[0], v[2] - v[0]), vref - v[0]) > 0)
                m_surfaceFace << faceInfo.first[0] << faceInfo.first[2] << faceInfo.first[1];
            else
                m_surfaceFace << faceInfo.first[0] << faceInfo.first[1] << faceInfo.first[2];
        }
        else // inner face
        {
            // the order of vertices is not important
            m_innerFace << faceInfo.first[0] << faceInfo.first[1] << faceInfo.first[2];
        }
    }

    m_surfaceVertices.resize(m_surfaceFace.size());
    m_surfaceNormals.resize(m_surfaceFace.size());

    m_innerVertices.resize(m_innerFace.size());
    m_innerNormals.resize(m_innerFace.size());

    UpdateVertexNormal();

    m_numVert = m_tetMesh->GetTopology()->GetNumVertices();
    m_forceBuffer.resize(m_numVert * 2);

	m_surfaceVertMask.clear();
	m_surfaceVertMask.resize(m_tetMesh->GetVertices().size(), 0);

	for(int i=0; i<m_surfaceFace.size(); i++)
	{
		int idex = m_surfaceFace[i];
		m_surfaceVertMask[idex] = 1;
	}

}

void GLTetMesh::OnSetForceScaleLevel(int scalelevel)
{
    m_scaleLevel = scalelevel;
}

void GLTetMesh::UpdateSurfaceArea(void)
{
	if (!m_tetMesh)
        return;
	double *vertices = reinterpret_cast<double *>(m_tetMesh->GetVertices().data());
	
	int n = m_surfaceFace.size();
	m_surfaceArea.resize(n);
    for (int i = 0; i < n; ++i)
    {
        int idx = m_surfaceFace[i] * 3;
        m_surfaceVertices[i] = QVector3D((float) vertices[idx], (float) vertices[idx + 1], (float) vertices[idx + 2]);

        if (i % 3 == 2) // updated a face
        {
            // compute normal
            double area = QVector3D::crossProduct(
                m_surfaceVertices[i - 1] - m_surfaceVertices[i - 2],
                m_surfaceVertices[i]     - m_surfaceVertices[i - 2]
			).length();

            m_surfaceArea[i]     = 
            m_surfaceArea[i - 1] = 
            m_surfaceArea[i - 2] = 0.5 * area;
        }
    }
}

void GLTetMesh::UpdateVertexNormal(void)
{
    if (!m_tetMesh)
        return;
    double *vertices = reinterpret_cast<double *>(m_tetMesh->GetVertices().data());

    int n = m_surfaceFace.size();
    for (int i = 0; i < n; ++i)
    {
        int idx = m_surfaceFace[i] * 3;
        m_surfaceVertices[i] = QVector3D((float) vertices[idx], (float) vertices[idx + 1], (float) vertices[idx + 2]);

        if (i % 3 == 2) // updated a face
        {
            // compute normal
            QVector3D normal = QVector3D::crossProduct(
                m_surfaceVertices[i - 1] - m_surfaceVertices[i - 2],
                m_surfaceVertices[i]     - m_surfaceVertices[i - 2]
            ).normalized();

            m_surfaceNormals[i]     = 
            m_surfaceNormals[i - 1] = 
            m_surfaceNormals[i - 2] = normal;
        }
    }

    n = m_innerFace.size();
    for (int i = 0; i < n; ++i)
    {
        int idx = m_innerFace[i] * 3;
        m_innerVertices[i] = QVector3D((float) vertices[idx], (float) vertices[idx + 1], (float) vertices[idx + 2]);

        if (i % 3 == 2) // updated a face
        {
            // compute normal
            QVector3D normal = QVector3D::crossProduct(
                m_innerVertices[i - 1] - m_innerVertices[i - 2],
                m_innerVertices[i]     - m_innerVertices[i - 2]
            ).normalized();

            m_innerNormals[i]     = 
            m_innerNormals[i - 1] = 
            m_innerNormals[i - 2] = normal;
        }
    }
}

void GLTetMesh::DrawTetMesh(void)
{    
    auto &shader = m_surfaceShader;

    shader.bind();
    m_glWidget->SetupProjViewModelMatrixForShader(shader);
    m_glWidget->SetupLightsForShader(shader);

    shader.enableAttributeArray("aVertex");
    shader.enableAttributeArray("aNormal");

    shader.setUniformValue("uObjectMaterial.diffuse"  , QVector4D(0.606f, 0.612f, 0.896f, 1.0f));
    shader.setUniformValue("uObjectMaterial.specular" , QVector4D(0.5f, 0.5f, 0.5f, 1.0f));
    shader.setUniformValue("uObjectMaterial.shininess", 10.0f);

    //glDisable(GL_DEPTH_TEST);
    //glEnable(GL_BLEND);
    //glBlendFunc(GL_ZERO, GL_SRC_COLOR);
    //glBlendFunc(GL_ONE, GL_ZERO);
    //glBlendFunc(GL_DST_COLOR, GL_ZERO);

    shader.setAttributeArray("aVertex", m_surfaceVertices.constData());
    shader.setAttributeArray("aNormal", m_surfaceNormals.constData());
    glDrawArrays(GL_TRIANGLES, 0, m_surfaceVertices.size());    

    shader.disableAttributeArray("aVertex");
    shader.disableAttributeArray("aNormal");
    shader.release();

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);

	if (m_vertForce)
		DrawVertForce();

}

void GLTetMesh::DrawVertForce(void)
{
    double *vertices = reinterpret_cast<double *>(m_tetMesh->GetVertices().data());
    // prepare buffer
    float scale = 0.0001 * exp((float)m_scaleLevel) * m_forceScale ;
    for (int i = 0; i < m_numVert; ++i)
    {
        m_forceBuffer[i * 2] = QVector3D((float) vertices[i * 3], (float) vertices[i * 3 + 1], (float) vertices[i * 3 + 2]);
        m_forceBuffer[i * 2 + 1] = m_forceBuffer[i * 2] + 
            scale * QVector3D((float) m_vertForce[i * 3], (float) m_vertForce[i * 3 + 1], (float) m_vertForce[i * 3 + 2]);
    }

    m_forceShader.bind();
    m_glWidget->SetupProjViewModelMatrixForShader(m_forceShader);
    m_forceShader.setUniformValue("color", QVector4D(0.6, 0.6, 0.2, 1.0));

    m_forceShader.enableAttributeArray("aVertex");
    m_forceShader.setAttributeArray("aVertex", m_forceBuffer.constData());
    glDrawArrays(GL_LINES, 0, m_forceBuffer.size());
    m_forceShader.disableAttributeArray("aVertex");
    m_forceShader.release();
}

void GLTetMesh::DrawVertices(void)
{
}
   
int GLTetMesh::GetSurfaceVertices(std::vector<float>& buf)
{
	//buf.clear();
	//for(int i=0; i<m_surfaceVertices.size(); i++)
	//{
	//	QVector3D vert = m_surfaceVertices[i];
	//	buf.push_back((float)vert.x());
	//	buf.push_back((float)vert.y());
	//	buf.push_back((float)vert.z());
	//}

	buf.clear();
	for(int i=0; i<m_tetMesh->GetVertices().size(); i++)
	{
		std::vector<Vector3d> vert = m_tetMesh->GetVertices();
		buf.push_back((float)vert[i].X());
		buf.push_back((float)vert[i].Y());
		buf.push_back((float)vert[i].Z());
	}

	return m_tetMesh->GetVertices().size();
}

int GLTetMesh::GetSurfaceFaces(std::vector<uint>& buf)
{
	buf.clear();
	for(int i=0; i<m_surfaceFace.size(); i++)
	{
		GLint ind = m_surfaceFace[i];
		buf.push_back((uint)ind);
	}
	
	return m_surfaceFace.size()/3;
}

