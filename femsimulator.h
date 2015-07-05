#ifndef FEMSIMULATOR_H
#define FEMSIMULATOR_H

#include <QtWidgets/QMainWindow>
#include "comHeader.h"
#include "ui_femsimulator.h"

#include "TetMesh.h"
#include "CorotatedLinearModel.h"
#include "CorotatedLinearModelMultiThreaded.h"
#include "FEMIntegrator.h"

#include "RestShapeProblem.h"
#include "TrajParameterProblem.h"
//#include "ImageBasedCollider.h"
#include <memory>

using namespace Libin;

class CentralGLWidget;
struct SimuTask;

class FEMSimulator : public QMainWindow
{
    Q_OBJECT

public:
    FEMSimulator(QWidget *parent = 0);
    ~FEMSimulator();

public:    
    void LoadConfig(const QString &confFileName);

signals:
    void signalSimulateTimeChanged(const QString &);
    void signalWorldTimeChanged(const QString &);
    void signalTimeStepChanged(const QString &);

    void signalTetNumberVerticesChanged(const QString &);
    void signalTetNumberElementsChanged(const QString &);
    void signalTetTotalMassChanged(const QString &);
    void signalTetDensityChanged(const QString &);

    void signalTetEChanged(const QString &);
    void signalTetEChanged(int);
    void signalTetNuChanged(const QString &);

    void signalRendered(void);

	void signalDump(void);

    void signalResultReady(void);

public slots:
    void OnLoadConfig(void);

    void OnRun(void);
    void OnStep(void);
    void OnPause(void);
    void OnStop(void);
    void OnDoOneStep(void);
    void OnLoadShape(void);
    void OnExportShape(void);
	void OnExportCamera(void);
	void OnLoadCamera(void);

    void OnTimestepChanged(const QString &text);
    void OnSlowRateChanged(int rate);

    void OnDumpFpsChanged(const QString &text);

    void OnDrawForceMethodChanged(int index);
    void OnDrawTotalForceCheckChanged(int state);
	void OnDrawWindVelocityCheckChanged(int state);
	void OnDrawWindForceCheckChanged(int state);
	void OnDrawContactForceCheckChanged(int state);

    void OnESliderChanged(int pos);

    void OnRenderTimeOut(void);

	void OnDump(void);
	void OnDumpCheckChanged(int state);

private:
    void Reset(void);

    void InitTetMesh(void);
    void InitModel(void);
    void InitIntegrator(void);
	void InitRestShapeProblem(void);
	void InitTrajParameterProblem(void);
    void InitMisc(void);
	void InitImgCD(void);

    void UpdateGLCenterView(void);
	void UpdateImgCD(void);
	void UpdataImgCDRestShapeProblem();
    void UpdateENu(double E, double nu);

	void DumpNode(std::vector<Vector3d>& nodalPosition,int index);

private:
    Ui::FEMSimulator ui;
    CentralGLWidget *m_centralGLWidget;
	//CImageBasedCollider *m_pImgCD;	

    QTimer m_renderTimer;
    QTimer m_idleTimer;

    int m_recordFps;
    int m_slowRate;
    double m_renderTimeStep;
    double m_timeStep;

    double m_renderTime;
    double m_simuTime;
    double m_worldTime;
    qint64 m_simuStartWorldTime;

    std::shared_ptr<TetMesh> m_tetMesh;
    std::shared_ptr<CorotatedLinearModel> m_model;
    std::shared_ptr<FEMIntegrator> m_integrator;

	std::shared_ptr<RestShapeProblem> m_restShapeProblem;

    std::vector<Vector3d> m_resetShape;

    Vector3d m_gravity;

    bool m_flagRunning;
    bool m_flagStopping;
    bool m_flagDumpingImg;

    QMap<QString, QString> m_configs;
    QString m_configFile;
    QString m_configPath;

    enum ProblemType
    {
        PT_SymplecticEuler,
        PT_BackwardEuler,
		PT_RestShapeProblem,
		PT_TrajParameterProblem
    };
    ProblemType m_problemType;

    enum DrawForceType
    {
        DFT_LastFrameElasticForce = 0,
        DFT_ResidualForce = 0,
        DFT_NodalForce,
        DFT_ExampleForce
    };
    DrawForceType m_drawForceType;

    bool m_flagDrawTotalForce;

    bool m_flagESliderChanging;
    QSlider *m_pESlider;
    double m_ESliderScale;
};

#endif // FEMSIMULATOR_H
