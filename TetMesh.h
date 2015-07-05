#pragma once

#include "StdStrings.h"
#include "VecMatInterface.h"
#include "comHeader.h"

#include <vector>
#include <array>
#include <memory>

namespace Libin
{
    class TetMeshTopology
    {
    public:
        struct VertInElm
        {
            int elm;
            int vidInElm;

            bool operator == (const VertInElm &rhs) { return elm == rhs.elm && vidInElm == rhs.vidInElm; }
            bool operator > (const VertInElm &rhs) { return elm > rhs.elm || ((elm == rhs.elm) && vidInElm > rhs.vidInElm); }
            bool operator < (const VertInElm &rhs) { return elm < rhs.elm || ((elm == rhs.elm) && vidInElm < rhs.vidInElm); }
        };
    public:
        TetMeshTopology(void);
        ~TetMeshTopology(void);

    public:
        bool LoadElements(const tstring &elmFile, int numLineToSkip = 0);
        void GatherGeometryInfo(void);

        int GetNumVertices(void) const { return m_nVertices; }
        int GetNumElement(void) const { return (int) m_elements.size(); }

        const std::vector<std::array<int, 4>> &GetElements(void) const { return m_elements; }
        const std::vector<VertInElm> &GetVertexNeighborElements(int vid) const { return m_vVertexNeighborElements[vid]; }
        const std::vector<unsigned int> &GetVertexNeighborVertices(int vid) const { return m_vVertexNeighborVertices[vid]; }

    private:
        int m_nVertices;
        std::vector<std::array<int, 4>> m_elements;

        // geometry properties
        //std::vector<std::vector<unsigned int>> m_vElementCommonFaceElements;
        //std::vector<std::vector<unsigned int>> m_vElementCommonEdgeElements;
        std::vector<std::vector<unsigned int>> m_vElementCommonVertexElements;

        // neighbor elements that share the face
        //std::vector<std::vector<unsigned int>> m_vFaceNeighborElements;

        // neighbor elements that contain the vertex
        // the first is the idx of the element
        // the second is the idx of the vertex in the element
        std::vector<std::vector<VertInElm>> m_vVertexNeighborElements;
        std::vector<std::vector<unsigned int>> m_vVertexNeighborVertices;
        //std::vector<std::vector<unsigned int>> m_vVertexCommonEdgeVertices;
    };
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    class TetMeshVertices
    {
    public:
        TetMeshVertices(void);
        ~TetMeshVertices(void);

    public:
        bool LoadVertices(const tstring &vertFile, int numLineToSkip = 0);
        static bool LoadVertices(const tstring &vertFile, std::vector<Vector3d> &vertices, int numLineToSkip = 0);

        bool SaveVertices(const tstring &vertFile);

        const std::vector<Vector3d> &GetVertices(void) const { return m_vertices; }
        std::vector<Vector3d> &GetVertices(void) { return m_vertices; }
        int GetNumVertices(void) const { return (int) m_vertices.size(); }

    private:
        std::vector<Vector3d> m_vertices;
    };    

    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////

    class TetMeshMaterials
    {
    public:        
        struct Material
        {
            double E;
            double nu;
            double density;

            double Lambda(void) const { return (nu * E) / ((1.0 + nu) * (1.0 - 2.0 * nu));  }
            double Mu(void) const { return E / (2.0 * (1.0 + nu)); }

            double LLambda(void) const { return E / ((1.0 + nu) * (1.0 - 2.0 * nu)); }
            double LAlpha(void) const { return LLambda() * nu; }

            void SetLambdaAlpha(double ll, double la) { 
                nu = la / ll;
                if (nu < 0.01)
                    nu = 0.01;
                if (nu > 0.49)
                    nu = 0.49;
                E = ll * ((1.0 + nu) * (1.0 - 2.0 * nu));
            }

			Material& operator=( const Material& m)
			{
				E = m.E;
				nu = m.nu;
				density = m.density;
				return *this;
			}
        };

    public:
        TetMeshMaterials(void);
        ~TetMeshMaterials(void);

    public:
        void SetUniformMaterial(double E, double nu, double density, int nelm);
        void SetUniformMaterial(const Material &mt, int nelm);
        const std::vector<Material> &GetMaterial(void) const { return m_elmMaterial; }

		void SetNonUniformMaterial(const std::vector<double>& weight, const std::vector<double>& ctrlE, const std::vector<double>& ctrlnu, double density, int nelm);

		const int GetNumCtrlPoint(void) const { return m_numCtrlPoint; }
		const std::vector<double> &GetWeightMatrix(void) const { return m_ctrlWeight; }
		const std::vector<double> &GetCtrlE(void) const { return m_ctrlE; }
		const std::vector<double> &GetCtrlNu(void) const { return m_ctrlNu; }

		void SetWeightMatrix( const std::vector<double>& weight) 
		{
			m_ctrlWeight.resize(weight.size());
			memcpy(&m_ctrlWeight[0], &weight[0], sizeof(double)*weight.size());
		}

    public:
        std::vector<Material> m_elmMaterial;

		int m_numCtrlPoint;

		std::vector<double> m_ctrlWeight;
		std::vector<double> m_ctrlE;
		std::vector<double> m_ctrlNu;

    };

    
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////

    class TetMesh
    {
    public:
        typedef TetMeshMaterials::Material TetMeshMaterial;

    private:
        TetMesh(const TetMesh &);
        TetMesh &operator = (const TetMesh);

    public:
        TetMesh(
            std::shared_ptr<TetMeshTopology> topology = std::shared_ptr<TetMeshTopology>(),
            std::shared_ptr<TetMeshVertices> vertices = std::shared_ptr<TetMeshVertices>(),
            std::shared_ptr<TetMeshMaterials> materials = std::shared_ptr<TetMeshMaterials>()
            );
        ~TetMesh(void);

    public:
        std::shared_ptr<TetMeshTopology> GetTopology(void) const { return m_topology; }
        std::shared_ptr<TetMeshVertices> GetVerticesContainer(void) const { return m_vertices; }
        std::shared_ptr<TetMeshMaterials> GetMaterialsContainer(void) const { return m_elmMaterialPtr; }

        bool LoadVertices(const tstring &vertFile, int numLineToSkip = 0) { return m_vertices->LoadVertices(vertFile, numLineToSkip); }
        bool LoadElements(const tstring &elmFile, int numLineToSkip = 0) { return m_topology->LoadElements(elmFile, numLineToSkip); }
    

        bool LoadTetMesh(const tstring &tetFile);
        bool LoadNodalMass(const tstring &massFile);

        void SetUniformMaterial(double E, double nu, double density) { m_elmMaterialPtr->SetUniformMaterial(E, nu, density, m_topology->GetNumElement()); }
        void SetUniformMaterial(const TetMeshMaterial &mt) { m_elmMaterialPtr->SetUniformMaterial(mt, m_topology->GetNumElement()); }
		
		void SetMateril(std::vector<double> &weight, std::vector<double>& E, std::vector<double>& nu, double density);
		void UpdateMaterial(std::vector<double>& E, std::vector<double>& nu);

        void ComputeVolumeMass(void);

        const std::vector<Vector3d> &GetVertices(void) const { return m_vertices->GetVertices(); }
        std::vector<Vector3d> &GetVertices(void) { return m_vertices->GetVertices(); }
        const std::vector<TetMeshMaterial> &GetMaterial(void) const { return m_elmMaterialPtr->GetMaterial(); }
		const TetMeshMaterials* GetMaterialPtr(void) const { return m_elmMaterialPtr.get(); }
        const std::vector<double> &GetElementVolume(void) const { return m_elmVolume; }
        const std::vector<double> &GetVertexMass(void) const { return m_vertMass; }
        
        const std::vector<std::array<int, 4>> &GetElements(void) const { return m_topology->GetElements(); }
        const std::vector<TetMeshTopology::VertInElm> &GetVertexNeighborElements(int vid) const { return m_topology->GetVertexNeighborElements(vid); }

        double GetVolume(void) const { return m_totalVolume; }
        double GetMass(void) const { return m_totalMass; }
        double GetDensity(void) const { return m_averageDensity; }

		void   SetSurfaceFaces(QVector<GLint> &faceIndex);
		const std::vector<int> &GetSurfaceFaces(void) const { return m_surfaceFaces; }

    private:
        std::shared_ptr<TetMeshTopology> m_topology;
        std::shared_ptr<TetMeshVertices> m_vertices;
	
        // material properties
        std::shared_ptr<TetMeshMaterials> m_elmMaterialPtr;

        std::vector<double> m_elmVolume;
        std::vector<double> m_vertMass;
		std::vector<int>	m_surfaceFaces;

        double m_totalVolume;
        double m_totalMass;
        double m_averageDensity;

        bool m_flagConstantMass;
    };

}