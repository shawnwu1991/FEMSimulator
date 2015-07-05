#include "stdafx.h"
#include "TetMesh.h"

#include <fstream>
#include <algorithm>
#include <numeric>

namespace Libin
{
    
    TetMeshTopology::TetMeshTopology(void)
        : m_nVertices(0)
    {
    }

    TetMeshTopology::~TetMeshTopology(void)
    {
    }
        
    bool TetMeshTopology::LoadElements(const tstring &elmFile, int numLineToSkip /*= 0*/)
    {
        tifstream fin(elmFile);
        if (!fin.is_open())
        {
            std::tcout << TEXT("Can't open ") << elmFile << std::endl;
            return false;
        }

        tstring line;
        while (numLineToSkip--)
            std::getline(fin, line);
        
        std::array<int, 4> elm;
        tstring prefix;

        m_elements.clear();
        int minId = INT_MAX;
        int maxId = 0;
        while (fin >> prefix >> elm[0] >> elm[1] >> elm[2] >> elm[3])
        {
            minId = std::min<int>(minId, *std::min_element(elm.begin(), elm.end()));
            maxId = std::max<int>(maxId, *std::max_element(elm.begin(), elm.end()));
            m_elements.push_back(elm);
        }

        // the index must start from 
        if (minId)
        {
            for (size_t i = 0; i < m_elements.size(); ++i)
            {
                auto &e = m_elements[i];
                for (int j = 0; j < 4; ++j)
                    e[j] -= minId;
            }
        }

        m_nVertices = maxId - minId + 1;

        GatherGeometryInfo();
        return true;
    }

    inline static TetMeshTopology::VertInElm MakeVertInElm(int el, int vid)
    {
        TetMeshTopology::VertInElm rt = {el, vid};
        return rt;
    }

    void TetMeshTopology::GatherGeometryInfo(void)
    {
        size_t nelm = m_elements.size();
        int nvert = m_nVertices;
        m_vElementCommonVertexElements.clear();
        m_vVertexNeighborElements.clear();

        //m_vElementCommonFaceElements.resize(nelm);
        //m_vElementCommonEdgeElements.resize(nelm);
        m_vElementCommonVertexElements.resize(nelm);
        m_vVertexNeighborElements.resize(nvert);
        m_vVertexNeighborVertices.resize(nvert);
        //m_vVertexCommonEdgeVertices.resize(nvert);
        //m_vFaceNeighborElements.resize(m_numFaces);

        /////////////////////////
        // prepare vertices' neighbor elements
        for (int el = 0; el < nelm; ++el)
        {
            for (int vt = 0; vt < 4; ++vt)
                m_vVertexNeighborElements[m_elements[el][vt]].push_back(MakeVertInElm(el, vt));
        }
    
        ///////////////////////
        ////// there shouldn't be any replicated element in m_vVertexNeighborElements
        for (int vt = 0; vt < nvert; ++vt)
        {
            std::sort(m_vVertexNeighborElements[vt].begin(), m_vVertexNeighborElements[vt].end());
            auto it = std::unique(m_vVertexNeighborElements[vt].begin(), m_vVertexNeighborElements[vt].end());
            if (it != m_vVertexNeighborElements[vt].end())
            {
                std::tcout << TEXT("Warning! Replicated element found @ vert ") << vt << std::endl;
                m_vVertexNeighborElements[vt].erase(it, m_vVertexNeighborElements[vt].end());
            }
        }

        //for (size_t eg = 0; eg < m_vTetEdges.size(); ++eg)
        //{
        //    auto &edge = m_vTetEdges[eg];
        //    m_vVertexCommonEdgeVertices[edge[0]].push_back(edge[1]);
        //    m_vVertexCommonEdgeVertices[edge[1]].push_back(edge[0]);
        //}
        //for (int vt = 0; vt < nvert; ++vt)
        //    std::sort(m_vVertexCommonEdgeVertices[vt].begin(), m_vVertexCommonEdgeVertices[vt].end());

        /////////////////////////
        // prepare elements' neighbors
        for (int el = 0; el < nelm; ++el)
        {
            for (int vt = 0; vt < 4; ++vt)
            {
                int vid = m_elements[el][vt];
                for (size_t i = 0; i < m_vVertexNeighborElements[vid].size(); ++i)
                {
                    auto &vneib = m_vVertexNeighborElements[vid][i];
                    m_vElementCommonVertexElements[el].push_back(vneib.elm);
                }
            }

            std::sort(m_vElementCommonVertexElements[el].begin(), m_vElementCommonVertexElements[el].end());
            m_vElementCommonVertexElements[el].erase(
                std::unique(m_vElementCommonVertexElements[el].begin(), m_vElementCommonVertexElements[el].end()), 
                m_vElementCommonVertexElements[el].end());
            m_vElementCommonVertexElements[el].erase(
                std::find(m_vElementCommonVertexElements[el].begin(), m_vElementCommonVertexElements[el].end(), el));
        }

        for (int vi = 0; vi < nvert; ++vi)
        {
            auto &vnbv = m_vVertexNeighborVertices[vi];
            auto &vnbe = m_vVertexNeighborElements[vi];

            std::set<unsigned int> vertSet;
            for (auto &ne : vnbe)
            {
                auto &elm = m_elements[ne.elm];
                for (auto &vt : elm)
                {
                    if (vt == vi)
                        continue;

                    vertSet.insert(vt);
                }
            }

            vnbv.assign(vertSet.begin(), vertSet.end());
        }
    
        //for (int el = 0; el < nelm; ++el)
        //{
        //    auto &vtNeighbor = m_vElementCommonVertexElements[el];
        //    std::array<int, 4> vts = m_elements[el];
        //    std::sort(vts.begin(), vts.end());
        //    int intsect[4];

        //    for (size_t nel = 0; nel < vtNeighbor.size(); ++nel)
        //    {
        //        std::array<int, 4> nvts = m_elements[nel];
        //        std::sort(nvts.begin(), nvts.end());

        //        size_t n = std::set_intersection(vts.begin(), vts.end(), nvts.begin(), nvts.end(), intsect) - intsect;
        //        switch (n)
        //        {
        //        case 2:
        //            m_vElementCommonEdgeElements[el].push_back(vtNeighbor[nel]);
        //            break;
        //        case 3:
        //            m_vElementCommonFaceElements[el].push_back(vtNeighbor[nel]);
        //            break;
        //        }
        //    }
        //}

        //////////////////////
        // prepare face's neighbors        
        //for (int el = 0; el < nelm; ++el)
        //{
        //    std::array<int, 4> vts = m_elements[el];

        //    m_vFaceNeighborElements[GetFaceIndex(vts[0], vts[1], vts[2])].push_back(el);
        //    m_vFaceNeighborElements[GetFaceIndex(vts[0], vts[2], vts[3])].push_back(el);
        //    m_vFaceNeighborElements[GetFaceIndex(vts[0], vts[3], vts[1])].push_back(el);
        //    m_vFaceNeighborElements[GetFaceIndex(vts[1], vts[3], vts[2])].push_back(el);
        //}
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    TetMeshVertices::TetMeshVertices(void)
    {
    }

    TetMeshVertices::~TetMeshVertices(void)
    {
    }

    bool TetMeshVertices::LoadVertices(const tstring &vertFile, int numLineToSkip /*= 0*/)
    {
        return LoadVertices(vertFile, m_vertices, numLineToSkip);
    }

    bool TetMeshVertices::LoadVertices(const tstring &vertFile, std::vector<Vector3d> &vertices, int numLineToSkip /*= 0*/)
    {
        tifstream fin(vertFile);
        if (!fin.is_open())
        {
            std::tcout << TEXT("Can't open ") << vertFile << std::endl;
            return false;
        }
        
        Vector3d vert;
        tstring prefix;
        tstring line;
        while (numLineToSkip--)
            std::getline(fin, line);

        vertices.clear();
        while (std::getline(fin, line))
        {
            size_t pos = 0;
            if ((pos = line.find_first_not_of(TEXT(" \n\r\b\t"))) == tstring::npos)
                continue;
            if (line[pos] == TEXT('#') || line[pos] == TEXT(';') || line[pos] == TEXT('/'))
                continue;

            tistringstream sstr(line);
            sstr >> prefix >> vert.X() >> vert.Y() >> vert.Z();
            if (!sstr)
            {
                vert.Z() = vert.Y();
                vert.Y() = vert.X();
                tstringstream(prefix) >> vert.X();
            }

            vertices.push_back(vert);
        }

        return true;
    }
      
    bool TetMeshVertices::SaveVertices(const tstring &vertFile)
    {
        tofstream fout(vertFile);
        if (!fout.is_open())
        {
            std::tcout << TEXT("Can't open ") << vertFile << std::endl;
            return false;
        }


        fout.precision(18);
		for(int i=0; i<m_vertices.size(); i++)
			fout << i << TEXT(" ") << m_vertices[i].X() << TEXT(" ") << m_vertices[i].Y() << TEXT(" ") << m_vertices[i].Z() << std::endl;

        return true;
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    TetMeshMaterials::TetMeshMaterials(void)
    {
		m_numCtrlPoint = 1;
    }

    TetMeshMaterials::~TetMeshMaterials(void)
    {
    }


    void TetMeshMaterials::SetUniformMaterial(double E, double nu, double density, int nelm)
    {
		m_numCtrlPoint = 1;

		m_ctrlE.resize(m_numCtrlPoint);
		m_ctrlNu.resize(m_numCtrlPoint);

		m_ctrlE[0] = E;
		m_ctrlNu[0] = nu;

        Material mt = { E, nu, density };
        SetUniformMaterial(mt, nelm);
    }

    void TetMeshMaterials::SetUniformMaterial(const Material &mt, int nelm)
    {	
		m_numCtrlPoint = 1;

		m_ctrlE.resize(m_numCtrlPoint);
		m_ctrlNu.resize(m_numCtrlPoint);
		m_ctrlWeight.resize(m_numCtrlPoint*nelm);

		m_ctrlE[0] = mt.E;
		m_ctrlNu[0] = mt.nu;
		for(int i=0; i<m_ctrlWeight.size(); i++)
			m_ctrlWeight[i] = 1;

        m_elmMaterial.resize(nelm);
        std::fill(m_elmMaterial.begin(), m_elmMaterial.end(), mt);
    }

	void TetMeshMaterials::SetNonUniformMaterial(const std::vector<double>& weight, const std::vector<double>& ctrlE, const std::vector<double>& ctrlnu, double density, int nelm)
	{
		m_numCtrlPoint = ctrlE.size();

		m_ctrlE.resize(m_numCtrlPoint);
		m_ctrlNu.resize(m_numCtrlPoint);
		m_ctrlWeight.resize(m_numCtrlPoint*nelm);

		memcpy(&m_ctrlE[0], &ctrlE[0], sizeof(double)*m_numCtrlPoint);
		memcpy(&m_ctrlNu[0], &ctrlnu[0], sizeof(double)*m_numCtrlPoint);
		memcpy(&m_ctrlWeight[0], &weight[0], sizeof(double)*m_numCtrlPoint*nelm);


		m_elmMaterial.resize(nelm);
		for(int i=0; i<nelm; i++)
		{
			double elmE = 0.0;
			double elmNu = 0.0;
			for(int j=0; j<m_numCtrlPoint; j++)
			{
				elmE += weight[i*m_numCtrlPoint+j]*ctrlE[j];
				//elmNu += weight[i*m_numCtrlPoint+j]*ctrlnu[j];
			}

			m_elmMaterial[i].E = elmE;
			m_elmMaterial[i].nu = ctrlnu[0]; 
			m_elmMaterial[i].density = density;
		}
	}


    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    TetMesh::TetMesh(
        std::shared_ptr<TetMeshTopology> topology /*= std::shared_ptr<TetMeshTopology>()*/,
        std::shared_ptr<TetMeshVertices> vertices /*= std::shared_ptr<TetMeshVertices>()*/,            
        std::shared_ptr<TetMeshMaterials> materials /*= std::shared_ptr<TetMeshMaterials>()*/
        )
        : m_topology(topology)
        , m_vertices(vertices)
        , m_elmMaterialPtr(materials)
        , m_totalVolume(0.0)
        , m_totalMass(0.0)
        , m_averageDensity(0.0)
        , m_flagConstantMass(false)
    {
        if (!m_topology)
            m_topology = std::make_shared<TetMeshTopology>();

        if (!m_vertices)
            m_vertices = std::make_shared<TetMeshVertices>();

        if (!m_elmMaterialPtr)
            m_elmMaterialPtr = std::make_shared<TetMeshMaterials>();      
    }


    TetMesh::~TetMesh(void)
    {
    }


    bool TetMesh::LoadTetMesh(const tstring &tetFile)
    {
        return false;
    }
   
    bool TetMesh::LoadNodalMass(const tstring &massFile)
    {
        tifstream fin(massFile);
        if (!fin.is_open())
        {
            std::tcout << TEXT("Can't open ") << massFile << std::endl;
            return false;
        }

        std::vector<double> mass;
        double m;
        while (fin >> m)
            mass.push_back(m);

        if (!GetVertices().empty() && mass.size() != GetVertices().size())
        {
            std::tcout << TEXT("Error! Need ") << GetVertices().size() << TEXT(" mass points, ") << mass.size() << TEXT(" are given.") << std::endl;
            return false;
        }

        m_totalMass = std::accumulate(mass.begin(), mass.end(), 0.0);
        m_vertMass = mass;
        if (m_totalVolume)
            m_averageDensity = m_totalMass / m_totalVolume;

        m_flagConstantMass = true;
        return true;
    }

	void TetMesh::SetMateril(std::vector<double> &weight, std::vector<double>& E, std::vector<double>& nu, double density)
	{
		if(E.size()==1)
			m_elmMaterialPtr->SetUniformMaterial(E[0], nu[0], density, m_topology->GetNumElement());
		else
			m_elmMaterialPtr->SetNonUniformMaterial(weight, E, nu, density, m_topology->GetNumElement());
	}

	void TetMesh::UpdateMaterial(std::vector<double>& E, std::vector<double>& nu)
	{
		if(E.size()==1)
			m_elmMaterialPtr->SetUniformMaterial(E[0], nu[0], m_elmMaterialPtr->GetMaterial().front().density, m_topology->GetNumElement());
		else
			m_elmMaterialPtr->SetNonUniformMaterial(m_elmMaterialPtr->GetWeightMatrix(), E, nu, m_elmMaterialPtr->GetMaterial().front().density, m_topology->GetNumElement());
	}


    void TetMesh::ComputeVolumeMass(void)
    {
        auto &elements = m_topology->GetElements();
        auto &vertices = m_vertices->GetVertices();
        auto &materials = m_elmMaterialPtr->GetMaterial();
        size_t nelm = elements.size();
        Matrix3d tetMat;

        m_vertMass.clear();
        m_vertMass.resize(vertices.size(), 0.0);

        m_elmVolume.clear();
        m_elmVolume.resize(nelm, 0.0);

        if (!m_flagConstantMass)
            m_totalMass = 0;

        m_totalVolume = 0;
        for (size_t el = 0; el < nelm; ++el)
        {
            auto &elm = elements[el];
            auto &mt = materials[el];

            tetMat.SetColumn(0, vertices[elm[1]] - vertices[elm[0]]);
            tetMat.SetColumn(1, vertices[elm[2]] - vertices[elm[0]]);
            tetMat.SetColumn(2, vertices[elm[3]] - vertices[elm[0]]);

            double elmVol = abs(tetMat.Determinant()) / 6.0;

            if (!m_flagConstantMass)
            {
                double elmMass = elmVol * mt.density;
                double vertElmMass = elmMass * 0.25;
                for (int vi = 0; vi < 4; ++vi)
                    m_vertMass[elm[vi]] += vertElmMass;

                m_totalMass += elmMass;
            }

            m_elmVolume[el] = elmVol;
            m_totalVolume += m_elmVolume[el];
        }

        m_averageDensity = m_totalMass / m_totalVolume;

        //std::ofstream fout("mass.txt");
        //fout.precision(18);
        //std::copy(m_vertMass.begin(), m_vertMass.end(), std::ostream_iterator<double>(fout, "\n"));
    }

	void TetMesh::SetSurfaceFaces(QVector<GLint> &faceIndex)
	{
		m_surfaceFaces.clear();
		m_surfaceFaces.resize(faceIndex.size());

		for(int i=0; i<faceIndex.size(); i++)
			m_surfaceFaces[i] = faceIndex[i];
	}
   

}