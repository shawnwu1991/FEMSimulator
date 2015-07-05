#include "stdafx.h"
#include "femsimulator.h"
#include "CentralGLWidget.h"

#include <QtCore\QString>

#include <numeric>
#include <functional>
#include <iterator>
using namespace Libin;

FEMSimulator::FEMSimulator(QWidget *parent)
	: QMainWindow(parent)
	, m_timeStep(0.0005)
	, m_slowRate(1)
	, m_simuTime(0)
	, m_worldTime(0)
	, m_simuStartWorldTime(0)
	, m_flagRunning(false)
	, m_flagStopping(false)
	, m_recordFps(30)
	, m_problemType(PT_BackwardEuler)
	, m_drawForceType(DFT_ExampleForce)
	, m_flagDrawTotalForce(false)
	, m_flagESliderChanging(false)
	, m_ESliderScale(100.0)
	, m_flagDumpingImg(false)
	, m_renderTimeStep(1.0/30.0)
	, m_renderTime(0.0)
{
	ui.setupUi(this);
	ui.statusBar->hide();

	m_centralGLWidget = new CentralGLWidget(ui.centralWidget);

	QVBoxLayout *centralWidgetLayout = new QVBoxLayout(ui.centralWidget);
	centralWidgetLayout->setObjectName(QStringLiteral("centralWidgetLayout"));
	centralWidgetLayout->setContentsMargins(0, 0, 0, 0);
	centralWidgetLayout->addWidget(m_centralGLWidget);

	connect(ui.actionE_xit, SIGNAL(triggered()), this, SLOT(close()));

#pragma region mainToolBar
	{
		/////////
		// buttons
		//ui.mainToolBar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
		ui.mainToolBar->addAction(QIcon(QStringLiteral(":/Images/Images/load.png")), tr("Load"), this, SLOT(OnLoadConfig()));
		ui.mainToolBar->addSeparator();

		ui.mainToolBar->addAction(QIcon(QStringLiteral(":/Images/Images/run.png")), tr("Run"), this, SLOT(OnRun()));
		ui.mainToolBar->addAction(QIcon(QStringLiteral(":/Images/Images/pause.png")), tr("Pause"), this, SLOT(OnPause()));
		ui.mainToolBar->addAction(QIcon(QStringLiteral(":/Images/Images/stop.png")), tr("Stop"), this, SLOT(OnStop()));
		ui.mainToolBar->addAction(QIcon(QStringLiteral(":/Images/Images/do_step.png")), tr("Step"), this, SLOT(OnStep()));
		ui.mainToolBar->addSeparator();
		ui.mainToolBar->addAction(QIcon(QStringLiteral(":/Images/Images/loadshape.png")), tr("Load Shape"), this, SLOT(OnLoadShape()));
		ui.mainToolBar->addAction(QIcon(QStringLiteral(":/Images/Images/export.png")), tr("Export Shape"), this, SLOT(OnExportShape()));
		ui.mainToolBar->addAction(QIcon(QStringLiteral(":/Images/Images/CameraIn.png")), tr("Inport Camera"), this, SLOT(OnLoadCamera()));
		ui.mainToolBar->addAction(QIcon(QStringLiteral(":/Images/Images/CameraOut.png")), tr("Export Camera"), this, SLOT(OnExportCamera()));
		ui.mainToolBar->addSeparator();

		// simulation speed
		QWidget *speedCtrl = new QWidget();    
		speedCtrl->setMaximumWidth(140);
		speedCtrl->setMinimumWidth(130);

		QBoxLayout *speedCtrlVertLayout = new QBoxLayout(QBoxLayout::TopToBottom, speedCtrl);

		QBoxLayout *speedCtrlRow1Layout = new QBoxLayout(QBoxLayout::LeftToRight);
		speedCtrlRow1Layout->addWidget(new QLabel(tr("Slow Rate:")));
		QSpinBox *speedSlowRate = new QSpinBox();
		speedSlowRate->setValue(m_slowRate);
		connect(speedSlowRate, SIGNAL(valueChanged(int)), this, SLOT(OnSlowRateChanged(int)));
		speedCtrlRow1Layout->addWidget(speedSlowRate);
		speedCtrlVertLayout->addLayout(speedCtrlRow1Layout);

		QBoxLayout *speedCtrlRow2Layout = new QBoxLayout(QBoxLayout::LeftToRight);
		speedCtrlRow2Layout->addWidget(new QLabel(tr("Timestep:")));
		QLineEdit *timeStep = new QLineEdit(QString::number(m_timeStep));
		timeStep->setAlignment(Qt::AlignRight);
		connect(this, SIGNAL(signalTimeStepChanged(const QString &)), timeStep, SLOT(setText(const QString &)));
		connect(timeStep, SIGNAL(textChanged(const QString &)), this, SLOT(OnTimestepChanged(const QString)));
		speedCtrlRow2Layout->addWidget(timeStep);
		speedCtrlVertLayout->addLayout(speedCtrlRow2Layout);

		ui.mainToolBar->addWidget(speedCtrl);
		ui.mainToolBar->addSeparator();

		// simulation/world time
		QWidget *simuRealTime = new QWidget();
		simuRealTime->setMaximumWidth(200);
		simuRealTime->setMinimumWidth(150);

		QGridLayout *simuRealTimeLayout = new QGridLayout(simuRealTime);
		simuRealTimeLayout->addWidget(new QLabel(tr("Simu time:")), 0, 0, Qt::AlignRight);
		simuRealTimeLayout->addWidget(new QLabel(tr("World time:")), 1, 0, Qt::AlignRight);

		QLineEdit *simuTime = new QLineEdit(QStringLiteral("0.000000"));
		QLineEdit *realTime = new QLineEdit(QStringLiteral("0.000000"));
		simuTime->setReadOnly(true);
		realTime->setReadOnly(true);
		simuTime->setAlignment(Qt::AlignRight);
		realTime->setAlignment(Qt::AlignRight);
		simuTime->resize(50, simuTime->size().height());
		realTime->resize(50, realTime->size().height());
		simuRealTimeLayout->addWidget(simuTime, 0, 1, Qt::AlignRight);
		simuRealTimeLayout->addWidget(realTime, 1, 1, Qt::AlignRight);

		connect(this, SIGNAL(signalSimulateTimeChanged(const QString &)), simuTime, SLOT(setText(const QString &)));
		connect(this, SIGNAL(signalWorldTimeChanged(const QString &)), realTime, SLOT(setText(const QString &)));
		ui.mainToolBar->addWidget(simuRealTime);
		ui.mainToolBar->addSeparator();
	}
#pragma endregion

#pragma region restShapeProblemToolBar
	{
		QWidget *forceCtrlWidget = new QWidget;
		QVBoxLayout *forceCtrlLayout = new QVBoxLayout(forceCtrlWidget);
		forceCtrlLayout->addWidget(new QLabel(tr("Draw force:")), 0, Qt::AlignLeft);
		QComboBox *forceType = new QComboBox();
		forceType->addItem(tr("Residual Force"));
		forceType->addItem(tr("Nodal Force"));
		forceType->addItem(tr("Example Force"));
		forceType->setMinimumWidth(120);
		connect(forceType, SIGNAL(activated(int)), this, SLOT(OnDrawForceMethodChanged(int)));
		forceCtrlLayout->addWidget(forceType, 0, Qt::AlignCenter);
		ui.restShapeToolBar->addSeparator();

		QCheckBox *drawTotalForce = new QCheckBox(tr("Draw total force"));
		drawTotalForce->setCheckState(Qt::Unchecked);
		connect(drawTotalForce, SIGNAL(stateChanged(int)), this, SLOT(OnDrawTotalForceCheckChanged(int)));
		forceCtrlLayout->addWidget(drawTotalForce);

		//QCheckBox *drawWindVelocity = new QCheckBox(tr("Draw Wind Velocity"));
		//drawWindVelocity->setCheckState(Qt::Checked);
		//connect(drawWindVelocity,SIGNAL(stateChanged(int)),this, SLOT(OnDrawWindVelocityCheckChanged(int)));
		//forceCtrlLayout->addWidget(drawWindVelocity);

		//QCheckBox *drawWindForce = new QCheckBox(tr("Draw Wind Force"));
		//drawWindForce->setCheckState(Qt::Checked);
		//connect(drawWindForce,SIGNAL(stateChanged(int)),this,SLOT(OnDrawWindForceCheckChanged(int)));
		//forceCtrlLayout->addWidget(drawWindForce);

		QCheckBox *drawContactForce = new QCheckBox(tr("Draw Contact Force"));
		drawContactForce->setCheckState(Qt::Checked);
		connect(drawContactForce,SIGNAL(stateChanged(int)),this,SLOT(OnDrawContactForceCheckChanged(int)));
		forceCtrlLayout->addWidget(drawContactForce);

		ui.restShapeToolBar->addWidget(forceCtrlWidget);
		ui.restShapeToolBar->addSeparator();

		QWidget *forceScale = new QWidget;
		QHBoxLayout *forceScaleLayout = new QHBoxLayout(forceScale);
		forceScaleLayout->addWidget(new QLabel(tr("Force scale:")), 0, Qt::AlignLeft);
		QSpinBox *spinScale = new QSpinBox();
		spinScale->setValue(0);
		connect(spinScale, SIGNAL(valueChanged(int)), m_centralGLWidget, SLOT(OnSetForceScaleLevel(int)));
		forceScaleLayout->addWidget(spinScale);

		ui.restShapeToolBar->addWidget(forceScale);
		ui.restShapeToolBar->addSeparator();

		QWidget *exportWidget = new QWidget;
		QVBoxLayout *exportParamLayout = new QVBoxLayout(exportWidget);

		QHBoxLayout *exportParamHLayout = new QHBoxLayout();
		exportParamLayout->addLayout(exportParamHLayout);

		QCheckBox *dumpcheck = new QCheckBox(tr("Dump"));
		connect(dumpcheck, SIGNAL(stateChanged(int)), this, SLOT(OnDumpCheckChanged(int)));
		exportParamHLayout->addWidget(dumpcheck, 0, Qt::AlignLeft);

		QLineEdit *dumpfps = new QLineEdit(QStringLiteral("30"));
		dumpfps->setMaximumWidth(50);
		connect(dumpfps, SIGNAL(textChanged(const QString &)), this, SLOT(OnDumpFpsChanged(const QString &)));
		exportParamHLayout->addWidget(dumpfps, 0, Qt::AlignLeft);

		ui.restShapeToolBar->addWidget(exportWidget);

		ui.restShapeToolBar->hide();
	}
#pragma endregion

#pragma region tetMeshStatToolBar
	{
		QWidget *vert = new QWidget();
		QHBoxLayout *vertLayout = new QHBoxLayout(vert);
		QLabel *vertLabel = new QLabel(tr("#vertices: 0"));
		connect(this, SIGNAL(signalTetNumberVerticesChanged(const QString &)), vertLabel, SLOT(setText(const QString &)));
		vertLayout->addWidget(vertLabel);
		ui.tetMeshStat->addWidget(vert);
		ui.tetMeshStat->addSeparator();

		QWidget *elm = new QWidget();
		QHBoxLayout *elmLayout = new QHBoxLayout(elm);
		QLabel *elmLabel = new QLabel(tr("#elements: 0"));
		connect(this, SIGNAL(signalTetNumberElementsChanged(const QString &)), elmLabel, SLOT(setText(const QString &)));
		elmLayout->addWidget(elmLabel);
		ui.tetMeshStat->addWidget(elm);
		ui.tetMeshStat->addSeparator();

		QWidget *mass = new QWidget();
		QHBoxLayout *massLayout = new QHBoxLayout(mass);
		QLabel *massLabel = new QLabel(tr("total mass: 0"));
		connect(this, SIGNAL(signalTetTotalMassChanged(const QString &)), massLabel, SLOT(setText(const QString &)));
		massLayout->addWidget(massLabel);
		ui.tetMeshStat->addWidget(mass);
		ui.tetMeshStat->addSeparator();

		QWidget *density = new QWidget();
		QHBoxLayout *densityLayout = new QHBoxLayout(density);
		QLabel *densityLabel = new QLabel(tr("density: 0"));
		connect(this, SIGNAL(signalTetDensityChanged(const QString &)), densityLabel, SLOT(setText(const QString &)));
		densityLayout->addWidget(densityLabel);
		ui.tetMeshStat->addWidget(density);
		ui.tetMeshStat->addSeparator();

		QWidget *E = new QWidget();
		QHBoxLayout *ELayout = new QHBoxLayout(E);
		QLabel *ELabel = new QLabel(tr("E: 0"));
		connect(this, SIGNAL(signalTetEChanged(const QString &)), ELabel, SLOT(setText(const QString &)));
		ELayout->addWidget(ELabel);
		ui.tetMeshStat->addWidget(E);
		ui.tetMeshStat->addSeparator();

		QWidget *ESlider = new QWidget();
		QHBoxLayout *esLayout = new QHBoxLayout(ESlider);
		m_pESlider = new QSlider(Qt::Horizontal);
		m_pESlider->setMinimum(300);
		m_pESlider->setMaximum(1000);
		connect(m_pESlider, SIGNAL(valueChanged(int)), this, SLOT(OnESliderChanged(int)));
		esLayout->addWidget(m_pESlider);
		ui.tetMeshStat->addWidget(ESlider);
		ui.tetMeshStat->addSeparator();

		QWidget *nu = new QWidget();
		QHBoxLayout *nuLayout = new QHBoxLayout(nu);
		QLabel *nuLabel = new QLabel(tr("nu: 0"));
		connect(this, SIGNAL(signalTetNuChanged(const QString &)), nuLabel, SLOT(setText(const QString &)));
		nuLayout->addWidget(nuLabel);
		ui.tetMeshStat->addWidget(nu);
		ui.tetMeshStat->addSeparator();


		ui.tetMeshStat->hide();
	}
#pragma endregion

	////////////////////////////////////////////////////////////
	connect(&m_renderTimer, SIGNAL(timeout()), this, SLOT(OnRenderTimeOut()));
	connect(&m_idleTimer, SIGNAL(timeout()), this, SLOT(OnDoOneStep()));

}

FEMSimulator::~FEMSimulator()
{

}

void FEMSimulator::OnLoadConfig(void)
{
	QString confFileName = QFileDialog::getOpenFileName(this, 
		tr("Load Simulation Config"), QString(), 
		tr("Config File (*.txt);;All files (*.*)"));

	if (confFileName.isEmpty())
		return;

	LoadConfig(confFileName);
}

void FEMSimulator::LoadConfig(const QString &confFileName)
{

	QFile confFile(confFileName);
	if (!confFile.open(QIODevice::ReadOnly))
	{       
		QMessageBox::warning(this, tr("Error!"), tr("Faile to open config file"));
		return;
	}

	QTextStream fin(&confFile);
	QString firstLine = fin.readLine().simplified();
	if (firstLine != QStringLiteral("!TetConfig"))
	{
		QMessageBox::warning(this, tr("Error!"), confFileName + tr(" is not a config file"));
		return;
	}

	m_configs.clear();
	while (!fin.atEnd())
	{
		QString line = fin.readLine();
		line = line.simplified();
		if (line[0] == QChar('#'))
			continue;

		if (line.isEmpty() || line[0].isSpace())
			continue;

		QStringList segs = line.split(':');
		if (segs.size() < 2)
			continue;
		m_configs[segs[0].toLower().simplified()] = segs[1].simplified();
	}

	m_configFile = confFileName;

	QString path = QDir::toNativeSeparators(m_configFile);
	m_configPath = path.left(path.lastIndexOf('\\') + 1);

	// gravity  
	m_gravity = Vector3d(0, -9.8, 0);
	QString gstr = m_configs.value(QStringLiteral("gravity"));
	if (!gstr.isEmpty())
	{
		std::wstringstream(gstr.utf16()) >> m_gravity.X() >> m_gravity.Y() >> m_gravity.Z();
	}

	QString type = m_configs.value(QStringLiteral("problemtype")).simplified();
	if (type == QStringLiteral("SymplecticEuler"))
		m_problemType = PT_SymplecticEuler;
	else if (type == QStringLiteral("BackwardEuler"))
		m_problemType = PT_BackwardEuler;
	else if (type == QStringLiteral("RestShapeProblem"))
		m_problemType = PT_RestShapeProblem;
	else if (type == QStringLiteral("TrajParameterProblem"))
		m_problemType = PT_TrajParameterProblem;
	else        
		m_problemType = PT_BackwardEuler;

	InitTetMesh();
	InitModel();

	if (m_problemType == PT_RestShapeProblem)
		InitRestShapeProblem();
	else if(m_problemType == PT_TrajParameterProblem)
		InitTrajParameterProblem();
	else
		m_restShapeProblem.reset();

	InitIntegrator();
	InitMisc();

	//InitImgCD();

	if(m_problemType == PT_TrajParameterProblem)
		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_pIntegrator = m_integrator;

}

void FEMSimulator::InitTrajParameterProblem(void)
{
	m_restShapeProblem = std::make_shared<TrajParameterProblem>(m_tetMesh->GetTopology());

	bool success = false;
	double timeStep = m_configs.value(QStringLiteral("sampletimestep_dyn")).toDouble(&success);
	if (!success)
	{
		QMessageBox::warning(this, tr("Warning!"), tr("Can't parse sample time step from config file!"));
		timeStep = 0.01;
	}

	// load static shape
	QString staticFrameName = m_configPath + m_configs.value(QStringLiteral("staticframe"));
	int numLineToSkip = m_configs.value(QStringLiteral("numlinetoskip")).toInt(&success);
	if (!success)
		numLineToSkip = 0;

	m_restShapeProblem->LoadStaticFrame(staticFrameName.utf16(), numLineToSkip, m_restShapeProblem->m_staticShape);

	// load motion trajactory
	auto prefixList = m_configs.value(QStringLiteral("prefix_dyn")).split(" ");
	auto suffixList = m_configs.value(QStringLiteral("suffix_dyn")).split(" ");
	auto digitWidthList = m_configs.value(QStringLiteral("digitwidth_dyn")).split(" ");
	auto stepList = m_configs.value(QStringLiteral("idxstep_dyn")).split(" ");
	auto numLineToSkipList = m_configs.value(QStringLiteral("numlinetoskip_dyn")).split(" ");
	auto starts = m_configs.value(QStringLiteral("startidx_dyn")).split(" ");
	auto ends = m_configs.value(QStringLiteral("endidx_dyn")).split(" ");

	int num = prefixList.size();
	for (int i = 0; i < prefixList.size(); ++i)
	{
		QString prefix = m_configPath + prefixList[i];
		QString suffix = suffixList[i];

		int digitWidth = 0;
		if(!digitWidthList.isEmpty())
			int digitWidth = digitWidthList[i].toInt(&success);

		int step = stepList[i].toInt(&success);
		if (!success)
			step = 1;
		numLineToSkip = numLineToSkipList[i].toInt(&success);
		if (!success)
			numLineToSkip = 0;

		int startidx = starts[i].toInt(&success);
		if (!success)
			startidx = 0;
		int endidx = ends[i].toInt(&success);
		if (!success)
			endidx = 0;

		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->LoadExample(prefix.utf16(), suffix.utf16(),
			digitWidth, startidx, step, endidx, numLineToSkip, timeStep);
	}


	int numVariable = m_configs.value(QStringLiteral("numoptvariable")).toInt(&success);
	if(!success)
		numVariable = 3;
	std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_numOptVariable = numVariable;
	std::cout << " number of optimized variable: " <<  numVariable << std::endl;

	auto UpperList = m_configs.value(QStringLiteral("upperbound")).split(" ");
	auto LowerList = m_configs.value(QStringLiteral("lowerbound")).split(" ");
	auto GradientStepList = m_configs.value(QStringLiteral("gradientstep")).split(" ");

	std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_upBound.resize(UpperList.size());
	std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_lowBound.resize(LowerList.size());
	std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_gradientStep.resize(GradientStepList.size());
	std::cout << " bound size: " <<  UpperList.size() << " " << LowerList.size() << " " << GradientStepList.size() << std::endl;

	for(int i=0; i<UpperList.size(); i++)
	{
		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_upBound[i] = UpperList[i].toDouble(&success);
		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_lowBound[i] = LowerList[i].toDouble(&success);
		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_gradientStep[i] = GradientStepList[i].toDouble(&success)/(UpperList[i].toDouble(&success)- LowerList[i].toDouble(&success));
	}

	m_restShapeProblem->SetMaterial(m_tetMesh->GetMaterialPtr());

	std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->Initialize( m_gravity, m_tetMesh->GetVertexMass(), m_tetMesh);
	std::cout << " TrajParameterProblem is initialized " << std::endl;
 
	// dump result or debug information
	auto optiOutputFile = m_configs.value(QStringLiteral("optioutput"));
	if (!optiOutputFile.isEmpty())
		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->InitOptiOutput(optiOutputFile);

	auto objOutputFile = m_configs.value(QStringLiteral("objfunoutput"));
	if (!objOutputFile.isEmpty())
	{
		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->InitObjFunOutput(objOutputFile);

		auto WSUpperList = m_configs.value(QStringLiteral("ws_upperbound")).split(" ");
		auto WSLowerList = m_configs.value(QStringLiteral("ws_lowerbound")).split(" ");
		
		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_upBoundWS.resize(WSUpperList.size());
		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_lowBoundWS.resize(WSLowerList.size());
		std::cout << " wsbound size: " <<  WSUpperList.size() << " " << WSLowerList.size()  << std::endl;

		
		for(int i=0; i<WSUpperList.size(); i++)
		{
			std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_upBoundWS[i] = WSUpperList[i].toDouble(&success);
			std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_lowBoundWS[i] = WSLowerList[i].toDouble(&success);
		}

		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_numSampleWS = m_configs.value(QStringLiteral("ws_samplenum")).toInt(&success);
		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_stiffnessDampWS = m_configs.value(QStringLiteral("ws_stiffnesscoef")).toDouble(&success);
		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_massDampWS = m_configs.value(QStringLiteral("ws_masscoef")).toDouble(&success);
	}

	auto wsOutputFile = m_configs.value(QStringLiteral("ws_output"));
	if (!wsOutputFile.isEmpty())
	{
		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->InitWSOutput(wsOutputFile);

		auto WSUpperList = m_configs.value(QStringLiteral("ws_upperbound")).split(" ");
		auto WSLowerList = m_configs.value(QStringLiteral("ws_lowerbound")).split(" ");
		
		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_upBoundWS.resize(WSUpperList.size());
		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_lowBoundWS.resize(WSLowerList.size());
		std::cout << " wsbound size: " <<  WSUpperList.size() << " " << WSLowerList.size()  << std::endl;

		
		for(int i=0; i<WSUpperList.size(); i++)
		{
			std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_upBoundWS[i] = WSUpperList[i].toDouble(&success);
			std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_lowBoundWS[i] = WSLowerList[i].toDouble(&success);
		}

		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_numSampleWS = m_configs.value(QStringLiteral("ws_samplenum")).toInt(&success);
		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_stiffnessDampWS = m_configs.value(QStringLiteral("ws_stiffnesscoef")).toDouble(&success);
		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->m_massDampWS = m_configs.value(QStringLiteral("ws_masscoef")).toDouble(&success);
	}
}

void FEMSimulator::InitRestShapeProblem(void)
{
	m_restShapeProblem = std::make_shared<RestShapeProblem>(m_tetMesh->GetTopology());

	bool success = false;
	// load static shape
	QString staticFrameName = m_configPath + m_configs.value(QStringLiteral("staticframe"));
	int numLineToSkip = m_configs.value(QStringLiteral("numlinetoskip")).toInt(&success);
	if (!success)
		numLineToSkip = 0;

	m_restShapeProblem->LoadStaticFrame(staticFrameName.utf16(), numLineToSkip, m_restShapeProblem->m_staticShape);

	m_restShapeProblem->SetMaterial(m_tetMesh->GetMaterialPtr());

	m_restShapeProblem->Initialize(m_gravity, m_tetMesh->GetVertexMass(), m_tetMesh);
}


//void FEMSimulator::InitImgCD(void)
//{
//	m_pImgCD = new CImageBasedCollider();
//
//	m_pImgCD->InitializeCollider();
//	m_pImgCD->ClearBBox();
//
//	int num_TetMesh = m_centralGLWidget->GetTetMeshNum();
//	printf("number of tetmesh: %d\n", num_TetMesh);
//	for (int i=0; i<num_TetMesh; i++)
//	{
//		std::vector<float> vert_buf;
//		std::vector<uint> face_buf;
//
//		QSharedPointer<GLTetMesh> pGLTetMesh = m_centralGLWidget->GetTetMesh(i);
//		int num_vert = pGLTetMesh->GetSurfaceVertices(vert_buf);
//		int num_face = pGLTetMesh->GetSurfaceFaces(face_buf);
//
//		printf("object: %d, %d\n", num_vert, num_face);
//		m_pImgCD->AddObject(num_vert, vert_buf, num_face, face_buf, i, true);	
//	}
//
//	printf("already load obj");
//
//	m_pImgCD->InitializePipeline();
//}

//void FEMSimulator::UpdateImgCD(void)
//{
//	m_pImgCD->ClearBBox();
//	int num_TetMesh = m_centralGLWidget->GetTetMeshNum();
//	for (int i=0; i<num_TetMesh; i++)
//	{
//		std::vector<float> vert_buf;
//
//		QSharedPointer<GLTetMesh> pGLTetMesh = m_centralGLWidget->GetTetMesh(i);
//		int num_vert = pGLTetMesh->GetSurfaceVertices(vert_buf);
//		m_pImgCD->UpdateObjectVert(num_vert, vert_buf,  m_tetMesh->GetVolume(), i);	
//	}
//
//	m_pImgCD->DoCollisionDetection();
//
//	for(int i=0; i<num_TetMesh; i++)
//	{
//		std::vector<Vector3d> contactForce(m_tetMesh->GetVertices().size());
//		std::vector<bool> contactFlag(m_tetMesh->GetVertices().size());
//		std::fill(contactForce.begin(),contactForce.end(),Vector3d::ZERO);
//		std::fill(contactFlag.begin(),contactFlag.end(),false);
//
//		m_pImgCD->ApplyContacts(i, contactForce, contactFlag);
//		m_integrator->AddExternalForce(contactForce);
//		m_centralGLWidget->setContactForce(contactForce);
//	}
//}


//void FEMSimulator::UpdataImgCDRestShapeProblem()
//{	
//	m_pImgCD->ClearBBox();
//	int num_TetMesh = m_centralGLWidget->GetTetMeshNum();
//	for (int i=0; i<num_TetMesh; i++)
//	{
//		std::vector<float> vert_buf;
//
//		QSharedPointer<GLTetMesh> pGLTetMesh = m_centralGLWidget->GetTetMesh(i);
//		int num_vert = pGLTetMesh->GetSurfaceVertices(vert_buf);
//		m_pImgCD->UpdateObjectVert(num_vert, vert_buf,  m_tetMesh->GetVolume(), i);	
//	}
//
//	m_pImgCD->DoCollisionDetection();
//
//	//for(int i=0; i<num_TetMesh; i++)
//	//{
//	//	std::vector<Vector3d> contactForce(m_tetMesh->GetVertices().size());
//	//	std::vector<bool> contactFlag(m_tetMesh->GetVertices().size());
//	//	std::fill(contactForce.begin(),contactForce.end(),Vector3d::ZERO);
//	//	std::fill(contactFlag.begin(),contactFlag.end(),false);
//
//	//	m_pImgCD->ApplyContacts(i, contactForce, contactFlag);
//
//	//	m_integrator->BackProjectRestShape(m_timeStep, contactFlag, contactForce);
//	//}
//}


void FEMSimulator::InitTetMesh(void)
{
	bool success = false;
	//////////////////////////////////////////////////////////////////
	// tet mesh
	m_tetMesh.reset(new TetMesh());

	if (!m_tetMesh->LoadElements((m_configPath + m_configs.value(QStringLiteral("elements"))).utf16()))
		return;
	if (!m_tetMesh->LoadVertices((m_configPath + m_configs.value(QStringLiteral("vertices"))).utf16()))
		return;


	//compute the center of the scene
	double xc,yc,zc;
	std::vector<double> temp_value(m_tetMesh->GetVertices().size());
	for (size_t i=0;i<m_tetMesh->GetVertices().size();++i)
	{
		temp_value[i] = m_tetMesh->GetVertices()[i].X();
	}
	std::sort(temp_value.begin(),temp_value.end());
	xc = (*temp_value.begin() + *(temp_value.end()-1))/2;

	for (size_t i=0;i<m_tetMesh->GetVertices().size();++i)
	{
		temp_value[i] = m_tetMesh->GetVertices()[i].Y();
	}
	yc = (*temp_value.begin() + *(temp_value.end()-1))/2;

	for (size_t i=0;i<m_tetMesh->GetVertices().size();++i)
	{
		temp_value[i] = m_tetMesh->GetVertices()[i].Z();
	}
	zc = (*temp_value.begin() + *(temp_value.end()-1))/2;
	//m_centralGLWidget->setWindInitialPos(xc,yc,zc);

	// load materail information
	int ctrlPointsNum = m_configs.value(QStringLiteral("ctrlpointnum")).toInt(&success);
	if(!success)
		ctrlPointsNum = 1;

	auto EList = m_configs.value(QStringLiteral("youngsmodulus")).split(" ");
	auto nuList = m_configs.value(QStringLiteral("possionsratio")).split(" ");
	double density = m_configs.value(QStringLiteral("density")).toDouble(&success);
	if (!success)
		density = 1000.0;

	std::vector<double> vecE;
	std::vector<double> vecNu;
	std::vector<double> vecWeight;

	for(int i=0; i<EList.size(); i++)
	{
		vecE.push_back(EList[i].toDouble(&success));
		vecNu.push_back(nuList[i].toDouble(&success));
	}

	//if(ctrlPointsNum != 1)
	{
		QString weighPath = m_configPath + m_configs.value(QStringLiteral("weight"));

		tifstream fin(weighPath.utf16());
		if (!fin.is_open())
		{
			std::tcout << TEXT("Can't open ") << weighPath.utf16() << std::endl;
			return;
		}

		tstring line;
		while (std::getline(fin, line))
		{
			tistringstream sstr(line);
			double tmpValue;
			for(int j=0; j<ctrlPointsNum; j++)
			{
				sstr >> tmpValue;
				vecWeight.push_back(tmpValue);
			}
		}
	}

	m_tetMesh->SetMateril(vecWeight, vecE, vecNu, density);
	UpdateENu(vecE[0], vecNu[0]);

	//double E = m_configs.value(QStringLiteral("youngsmodulus")).toDouble(&success);
	//if (!success)
	//	E = 1e7;
	//double nu = m_configs.value(QStringLiteral("possionsratio")).toDouble(&success);
	//if (!success)
	//	nu = 0.45;
	//double density = m_configs.value(QStringLiteral("density")).toDouble(&success);
	//if (!success)
	//	density = 1000.0;
	//m_tetMesh->SetUniformMaterial(E, nu, density);

	m_tetMesh->ComputeVolumeMass();

	if (!m_configs.value(QStringLiteral("nodalmass")).isEmpty())
		m_tetMesh->LoadNodalMass((m_configPath + m_configs.value(QStringLiteral("nodalmass"))).utf16());

	emit signalTetNumberVerticesChanged(tr("#vertices: %1").arg(m_tetMesh->GetVertices().size()));
	emit signalTetNumberElementsChanged(tr("#elements: %1").arg(m_tetMesh->GetElements().size()));
	emit signalTetTotalMassChanged(tr("total mass: %1").arg(m_tetMesh->GetMass()));
	emit signalTetDensityChanged(tr("density: %1").arg(m_tetMesh->GetDensity()));

	//UpdateENu(E, nu);
	ui.tetMeshStat->show();
	ui.restShapeToolBar->show();
}

void FEMSimulator::InitModel(void)
{
	bool success = false;
	////////////////////////////////////////////////////////////////
	// modal
	int nthread = m_configs.value(QStringLiteral("nthread")).toInt(&success);
	if (!success)
	{
		nthread = QThread::idealThreadCount();
		//printf("%d\n", nthread);
	}

	m_model.reset(new CorotatedLinearModelMultiThreaded(m_tetMesh->GetTopology(), nthread));
	//m_model.reset(new CorotatedLinearModel(m_tetMesh.get()));

	m_model->InitializeFirstPiolaKirchhoffMethod(m_tetMesh.get());
	m_model->InitializeUndeformedStiffnessMatrix(m_tetMesh.get());
}

void FEMSimulator::InitIntegrator(void)
{
	bool success = false;
	/////////////////////////////////////////////////////////////
	// integrator
	double dampingStiffnessCoef = m_configs.value(QStringLiteral("dampingstiffnesscoef")).toDouble(&success);
	if (!success)
		dampingStiffnessCoef = 0.0;
	double dampingMassCoef = m_configs.value(QStringLiteral("dampingmasscoef")).toDouble(&success);
	if (!success)
		dampingMassCoef = 0.0;


	m_integrator.reset(new FEMIntegrator(m_model.get(), m_tetMesh.get(), m_restShapeProblem));
	m_integrator->SetDampingParameters(dampingStiffnessCoef, dampingMassCoef);
	m_integrator->SetGravity(m_gravity);

	auto constrainedRange = m_configs.value(QStringLiteral("constrainedrange"));
	if (!constrainedRange.isEmpty())
	{
		auto rangeStrList = constrainedRange.split(' ');
		double range[6] = { 
			rangeStrList.value(0).toDouble(), rangeStrList.value(1).toDouble(), rangeStrList.value(2).toDouble(),
			rangeStrList.value(3).toDouble(), rangeStrList.value(4).toDouble(), rangeStrList.value(5).toDouble()
		};

		m_integrator->ConstrainNodesInRange(range, m_tetMesh.get());
	}

	auto constrainedIdsStr = m_configs.value(QStringLiteral("constrainednodes"));
	if (!constrainedIdsStr.isEmpty())
	{
		auto rangeStrList = constrainedIdsStr.split(' ');
		std::set<int> nodeset;
		for (auto &str : rangeStrList)
		{
			int id = str.toInt(&success);
			if (success)
				nodeset.insert(id);
		}

		std::vector<int> nodes(nodeset.begin(), nodeset.end());
		//std::vector<int> nodes(++nodeset.begin(), nodeset.end());
		//nodes.push_back(*nodeset.begin());

		////std::copy(nodes.begin(), nodes.end(), std::ostream_iterator<int>(std::cout, "\t"));
		////std::cout << std::endl;

		//if (!nodes.empty())
		//{
		//	for (auto &id : nodes)
		//		id += nodes.back();
		//	nodes.pop_back();
		//}

		m_integrator->ConstrainNodesByIndex(nodes);
	}

	m_integrator->InitAnimation(m_configPath);


	auto controledIdsStr = m_configs.value(QStringLiteral("controlednodes"));
	if (!controledIdsStr.isEmpty())
	{
		auto rangeStrList = controledIdsStr.split(' ');
		std::set<int> nodeset;
		for (auto &str : rangeStrList)
		{
			int id = str.toInt(&success);
			if (success)
				nodeset.insert(id);
		}

		std::vector<int> nodes(nodeset.begin(), nodeset.end());
		m_integrator->ControledNodesByIndex(nodes);
	}



	auto addforces = m_configs.value(QStringLiteral("addforces")).split(" ");
	std::vector<int> addForceVert;
	std::vector<Vector3d> addForcesF;
	for (int i = 0; i < addforces.size(); i += 4)
	{
		int vid = addforces[i].toInt(&success);
		if (!success)
			continue;
		Vector3d f;
		f[0] = addforces[i + 1].toDouble(&success);
		if (!success)
			continue;
		f[1] = addforces[i + 2].toDouble(&success);
		if (!success)
			continue;
		f[2] = addforces[i + 3].toDouble(&success);
		if (!success)
			continue;

		addForceVert.push_back(vid);
		addForcesF.push_back(f);
	}
	m_integrator->AddForce(addForceVert, addForcesF);
}


void FEMSimulator::OnDrawForceMethodChanged(int index)
{
	m_drawForceType = (DrawForceType) index;
	UpdateGLCenterView();
}

void FEMSimulator::InitMisc(void)
{
	bool success = false;
	m_centralGLWidget->ClearTetMesh();

	m_centralGLWidget->AddTetMesh(QSharedPointer<GLTetMesh>(
		new GLTetMesh(m_centralGLWidget, m_tetMesh)
		));

	if (!m_configs.value(QStringLiteral("resetshape")).isEmpty())
	{
		TetMeshVertices::LoadVertices(
			(m_configPath + m_configs.value(QStringLiteral("resetshape"))).utf16(), 
			m_resetShape);
	}
	else
		m_resetShape = m_tetMesh->GetVertices();

	m_timeStep = m_configs.value(QStringLiteral("timestep")).toDouble(&success);
	if (!success)
		m_timeStep = 0.0005;
	char buf[100];
	sprintf_s(buf, "%g", m_timeStep);
	QString strTimeStep(buf);
	emit signalTimeStepChanged(strTimeStep);

	OnStop();
}

void FEMSimulator::OnRun(void)
{
	if (!m_tetMesh)
		return;

	if (m_flagRunning)
	{
		OnPause();
		return;
	}

	m_simuStartWorldTime = QDateTime::currentMSecsSinceEpoch();
	m_renderTimer.start(33);
	m_idleTimer.start(0);
	m_flagRunning = true;
}

void FEMSimulator::OnStep(void)
{
	if (!m_tetMesh)
		return;

	m_flagRunning = true;
	m_simuStartWorldTime = QDateTime::currentMSecsSinceEpoch();
	OnDoOneStep();
	OnRenderTimeOut();
	m_flagRunning = false;
}

static int iteration_index = 1;
static QString fileName;
static int test_index = 0;
void FEMSimulator::OnDoOneStep(void)
{
	switch (m_problemType)
	{
	case PT_SymplecticEuler:
		m_integrator->SymplecticEulerStep(m_timeStep);
		m_tetMesh->GetVertices() = m_integrator->GetNodalPosition();
		break;

	case PT_BackwardEuler:
		m_integrator->ClearExternalForce();
		//UpdateImgCD();
		m_integrator->BackwardEulerStep(m_timeStep);
		m_tetMesh->GetVertices() = m_integrator->GetNodalPosition();
		DumpNode(m_tetMesh->GetVertices(),iteration_index);
		/*fileName = "FrameTwist/shape_" + QString::number(iteration_index) + ".txt";
		m_tetMesh->GetVerticesContainer()->SaveVertices(fileName.utf16());*/
		break;

	case PT_RestShapeProblem:
		if(test_index<200)
		{
			m_integrator->ClearExternalForce();
			m_integrator->BackwardEulerStepRestShape(m_timeStep);
			m_tetMesh->GetVertices() = m_integrator->GetNodalPosition();
			//DumpNode(m_tetMesh->GetVertices(),iteration_index);
			printf("force residual: %d, %g\n",iteration_index, m_integrator->ComputeTotalForceResidual());
			test_index++;
		}
		break;

	case PT_TrajParameterProblem:
		m_integrator->ClearExternalForce();
		std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->UpdateNLopt();
		m_tetMesh->GetVertices() = m_integrator->GetNodalPosition();

		//static int tmpIndx = 0;
		//if(tmpIndx<80)
		//{
		//   m_tetMesh->GetVertices() = std::dynamic_pointer_cast<TrajParameterProblem>(m_restShapeProblem)->GetDynExampleShape(tmpIndx)->GetVertices();
		//   tmpIndx++;
		//}

		break;
	}

	m_simuTime += m_timeStep;
	if (m_simuTime > (m_renderTime-(1e-9)))
	{
		m_flagDumpingImg = true;
		m_renderTime += m_renderTimeStep;
	}

	if(m_flagDumpingImg)
	{
		UpdateGLCenterView();
		m_centralGLWidget->DumpImage();
	}

	iteration_index++;
}


void FEMSimulator::OnPause(void)
{
	if (!m_tetMesh)
		return;

	if (!m_flagRunning)
	{
		if (m_worldTime > 0)
			OnRun();
		return;
	}

	m_renderTimer.stop();
	m_idleTimer.stop();
	OnRenderTimeOut();
	m_flagRunning = false;
}

void FEMSimulator::OnStop(void)
{
	if (!m_tetMesh)
		return;

	m_flagStopping = true;
	m_flagRunning = false;

	m_simuTime = 0;
	m_worldTime = 0;
	m_renderTime = 0;
	m_renderTimer.stop();
	m_idleTimer.stop();
	Reset();

	OnRenderTimeOut();
	m_flagStopping = false;
}

void FEMSimulator::OnLoadShape(void)
{    
	QString vertFileName = QFileDialog::getOpenFileName(this, 
		tr("Load Shape"), QString(), 
		tr("Vertex File (*.txt);;All files (*.*)"));

	if (vertFileName.isEmpty())
		return;

	m_tetMesh->GetVerticesContainer()->LoadVertices(vertFileName.utf16());
	m_integrator->SetState(m_tetMesh->GetVertices());
	m_centralGLWidget->UpdateTetMesh();
	m_centralGLWidget->GetTetMesh(0)->SetVertForce(NULL, 1.0 / m_tetMesh->GetMass() * m_tetMesh->GetVertices().size());
	m_centralGLWidget->update();
}

void FEMSimulator::OnLoadCamera(void)
{    
	QString cameraFileName = QFileDialog::getOpenFileName(this, 
		tr("Load Camera"), QString(), 
		tr("Camera File (*.txt);;All files (*.*)"));

	if (cameraFileName.isEmpty())
		return;

	m_centralGLWidget->LoadCamera(cameraFileName.utf16());
	m_centralGLWidget->update();
}


void FEMSimulator::OnExportCamera(void)
{
	QString camFileName = QFileDialog::getSaveFileName(this, 
		tr("Export Current Camera"), QString(), 
		tr("Vertex File (*.txt);;All files (*.*)"));

	if (camFileName.isEmpty())
		return;

	m_centralGLWidget->SaveCamera(camFileName.utf16());
}


void FEMSimulator::OnExportShape(void)
{    
	QString vertFileName = QFileDialog::getSaveFileName(this, 
		tr("Export Current Shape"), QString(), 
		tr("Vertex File (*.vert);;All files (*.*)"));

	if (vertFileName.isEmpty())
		return;

	m_tetMesh->GetVerticesContainer()->SaveVertices(vertFileName.utf16());
}


void FEMSimulator::OnDumpFpsChanged(const QString &text)
{
	bool success = false;
	int fps = text.toInt(&success);
	if (success)
	{
		m_renderTimeStep = 1.0 / (double)fps;
	}
}

void FEMSimulator::OnTimestepChanged(const QString &text)
{
	bool success = false;
	double timestep = text.toDouble(&success);
	if (success)
		m_timeStep = timestep;
}


void FEMSimulator::OnSlowRateChanged(int rate)
{
	m_slowRate = rate;
}

void FEMSimulator::OnDrawTotalForceCheckChanged(int state)
{
	m_flagDrawTotalForce = (state != Qt::Unchecked);
	UpdateGLCenterView();
}

void FEMSimulator::OnDrawWindVelocityCheckChanged(int state)
{
	m_centralGLWidget->setDrawWindVelocity((state != Qt::Unchecked));
	m_centralGLWidget->update();
}

void FEMSimulator::OnDrawWindForceCheckChanged(int state)
{
	m_centralGLWidget->setDrawWindForce((state != Qt::Unchecked));
	m_centralGLWidget->update();
}

void FEMSimulator::OnDrawContactForceCheckChanged(int state)
{
	m_centralGLWidget->setDrawContactForce((state != Qt::Unchecked));
	m_centralGLWidget->update();
}


void FEMSimulator::Reset(void)
{
	if (!m_tetMesh)
		return;

	m_tetMesh->GetVertices() = m_resetShape;
	m_integrator->SetState(m_resetShape);
	m_integrator->ClearExternalForce();
}

void FEMSimulator::UpdateENu(double E, double nu)
{
	emit signalTetEChanged(tr("E: %1").arg(E, 0, 'e'));
	emit signalTetNuChanged(tr("nu: %1").arg(nu));

	if (!m_flagESliderChanging)
	{
		m_flagESliderChanging = true;
		m_pESlider->setValue((int) (m_ESliderScale * log10(E)));
		m_flagESliderChanging = false;
	}


	m_centralGLWidget->DrawTextLine(QString("E: %1    nu: %2").arg(E, 10).arg(nu, 6));
}

void FEMSimulator::OnESliderChanged(int pos)
{
	if (m_flagESliderChanging)
		return;

	if (!m_tetMesh)
		return;

	m_flagESliderChanging = true;
	auto mt = m_tetMesh->GetMaterial().front();
	mt.E =  pow(10.0, (double)pos / m_ESliderScale);
	m_tetMesh->SetUniformMaterial(mt);

	UpdateENu(mt.E, mt.nu);
	m_flagESliderChanging = false;

	
	//m_flagESliderChanging = true;
	//double curMassDamping = m_integrator->GetMassDamping();
	//double curStiffnessDamping = m_integrator->GetStiffnessDamping();

	//curStiffnessDamping = 0.0 + 0.3*(double)(pos-300) / (double)700;
	//m_integrator->SetDampingParameters(curStiffnessDamping, curMassDamping);
	//std::cout << pos << " " << m_ESliderScale << " " << curMassDamping<< std::endl;
	//UpdateENu(curMassDamping, curStiffnessDamping);
	//m_flagESliderChanging = false;

	Reset();
}

void FEMSimulator::UpdateGLCenterView(void)
{    
	m_centralGLWidget->UpdateTetMesh();

	if (m_flagDrawTotalForce)
		m_centralGLWidget->GetTetMesh(0)->SetVertForce(reinterpret_cast<const double *>(m_integrator->GetLastStepNodalForce().data()), 1.0 / m_tetMesh->GetMass() * m_tetMesh->GetVertices().size());
		//m_centralGLWidget->GetTetMesh(0)->SetVertForce(reinterpret_cast<const double *>(m_integrator->GetExternalForce().data()), 1.0 / m_tetMesh->GetMass() * m_tetMesh->GetVertices().size());
	else
		m_centralGLWidget->GetTetMesh(0)->SetVertForce(NULL, 1.0);

	//m_centralGLWidget->GetTetMesh(0)->SetVertForce(reinterpret_cast<const double *>(m_integrator->GetExternalForce().data()), 1000000.0);

	m_centralGLWidget->update();

}

void FEMSimulator::OnRenderTimeOut(void)
{
	qint64 curWorldTime = QDateTime::currentMSecsSinceEpoch();
	if (!m_flagStopping)
		m_worldTime += (double)(curWorldTime - m_simuStartWorldTime) / 1000.0;
	m_simuStartWorldTime = curWorldTime;

	UpdateGLCenterView();

	char buf[100];
	sprintf_s(buf, "%0.6f", m_simuTime);
	QString strSimuTime(buf);
	sprintf_s(buf, "%0.6f", m_worldTime);
	QString strWorldTime(buf);

	emit signalSimulateTimeChanged(strSimuTime);
	emit signalWorldTimeChanged(strWorldTime);
	emit signalRendered();
}


void FEMSimulator::OnDumpCheckChanged(int state)
{
	m_flagDumpingImg = state == Qt::Checked;

	if (state == Qt::Checked)
		connect(this, SIGNAL(signalDump()), this, SLOT(OnDump()));
	else
		disconnect(this, SIGNAL(signalDump()), this, SLOT(OnDump()));
}

void FEMSimulator::OnDump(void)
{
	m_centralGLWidget->DumpImage();
}


void FEMSimulator::DumpNode(std::vector<Vector3d>& nodalPosition,int frameIndex)
{
	QString format = ".node";
	QString example_fileName = "Node/dump_"  + QString::number(frameIndex) + format;
	QString example_filePath = QDir::toNativeSeparators(example_fileName);

	// export example frame
	tofstream fout(example_filePath.utf16());
	fout.precision(18);
	fout << nodalPosition.size() << TEXT(" 3 0 0") << std::endl;
	for(size_t i=0; i<nodalPosition.size(); i++)
		fout  << i << TEXT(" ") << nodalPosition[i].X() << TEXT(" ") << nodalPosition[i].Y() << TEXT(" ") << nodalPosition[i].Z() << std::endl;
}