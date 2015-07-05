#include "stdafx.h"
#include "TrajParameterProblem.h"
#include "TetMesh.h"
#include "CorotatedLinearModelMultiThreaded.h"

#include <iomanip>
#include <numeric>
#include <algorithm>
#include <functional>
#include <ctime>


namespace Libin
{
	static int optimizeIteration = 0;

	double callbackFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
	{
		TrajParameterProblem* ptr = reinterpret_cast<TrajParameterProblem *>(my_func_data);

		// set material property
		std::vector<double> curValue(ptr->m_numOptVariable); 
		ptr->DecodeVariable(x, curValue);

		//// retrive the referecen shape
		if (!ptr->RetriveReferenceShape(curValue, ptr->m_staticShape->GetVertices(), ptr->m_refShape))
			return 100000.0;

		// get the objective function value
		double objValue = ptr->MatchTrajectory(curValue);

		std::cout << "objValue " << objValue << std::endl;

		// get the gradient value
		if(grad.size()!=0)
		{
			std::vector<double> new_x(x.size());
			for(int i=0; i<ptr->m_numOptVariable; i++)
			{
				memcpy(&new_x[0], &x[0], sizeof(double)*x.size());
				new_x[i] = new_x[i] + ptr->m_gradientStep[i];
				ptr->DecodeVariable(new_x, curValue);

				if (!ptr->RetriveReferenceShape(curValue, ptr->m_staticShape->GetVertices(), ptr->m_refShape))
					return 100000.0;

				// get the objective function value
				double tmp1 = ptr->MatchTrajectory(curValue);

				memcpy(&new_x[0], &x[0], sizeof(double)*x.size());
				new_x[i] = new_x[i] - ptr->m_gradientStep[i];
				ptr->DecodeVariable(new_x, curValue);

				if (!ptr->RetriveReferenceShape(curValue, ptr->m_staticShape->GetVertices(), ptr->m_refShape))
					return 100000.0;

				// get the objective function value
				double tmp2 = ptr->MatchTrajectory(curValue);

				grad[i] = (tmp1 - tmp2)/(2*ptr->m_gradientStep[i]);

				printf("gradient: %g\n", grad[i]);

			}
		}

		optimizeIteration++;

		if(ptr->m_bOutputOpti)
		{
			ptr->fout_opti << optimizeIteration << " " << objValue << " ";
			for(int i=0; i<ptr->m_numOptVariable; i++)
				ptr->fout_opti << x[i] << " " << curValue[i] << " " ;
			ptr->fout_opti << std::endl;
		}

		return objValue;
	}

	void TrajParameterProblem::WarmStartSampleFast()
	{
		// store the freq difference for each line search sample point
		std::vector<double> freqDiff_LineSearch(m_numSampleWS*m_materials->m_numCtrlPoint);

		std::vector<int> preSampleWS(m_materials->m_numCtrlPoint);
		std::fill(preSampleWS.begin(), preSampleWS.end(), int((double)m_numSampleWS/2.0));
		
		fout_WS<< " init value: ";
		for(int i=0; i<preSampleWS.size(); i++)
			fout_WS << preSampleWS[i] << " " ;
		fout_WS << std::endl;

		std::map<double, int> covs;
		for(int i=0; i<m_materials->m_numCtrlPoint; i++)
		{
			double tmpCOV = WarmStartLineSearch(i, preSampleWS, freqDiff_LineSearch);
			// std::map use descene order to sort, so need minus in front
			covs.insert(std::pair<double, int>(-tmpCOV, i));
		}

		std::map<double, int>::iterator iter;
		for(iter=covs.begin(); iter!=covs.end(); iter++)
			fout_WS << "cov: " << iter->second << " " << -1*iter->first << std::endl;

		int iterationNum = 0;
		int idxDiff = 0;
		std::vector<int> tmpSampleWS(m_materials->m_numCtrlPoint);
		do{
			
			for(iter=covs.begin(); iter!=covs.end(); iter++)
			{
				tmpSampleWS = preSampleWS;

				double tmpCOV = WarmStartLineSearch(iter->second, preSampleWS, freqDiff_LineSearch);
				int tmpIdx = FindMinSample(iter->second*m_numSampleWS, freqDiff_LineSearch);
				fout_WS << " find minimu of " << iter->second << " at " << tmpIdx << std::endl; 
				if(tmpIdx != preSampleWS[iter->second])
					preSampleWS[iter->second] = tmpIdx + int((double)(preSampleWS[iter->second]-tmpIdx)/2.0);
			}

			fout_WS << iterationNum << " iteration result: ";
			for(int i=0; i<preSampleWS.size(); i++)
				fout_WS << preSampleWS[i] << " " ;
			fout_WS << std::endl;

			iterationNum++;

			idxDiff = 0;
			for(int i=0; i<	m_materials->m_numCtrlPoint; i++)
				idxDiff += abs(tmpSampleWS[i]-preSampleWS[i]);

		} while(idxDiff!=0);
	}

	void TrajParameterProblem::WarmStartSample()
	{
		// store the freq difference for each line search sample point
		std::vector<double> freqDiff_LineSearch(m_numSampleWS*m_materials->m_numCtrlPoint);

		std::vector<int> preSampleWS(m_materials->m_numCtrlPoint);
		std::fill(preSampleWS.begin(), preSampleWS.end(), int((double)m_numSampleWS/2.0));
		
		fout_WS<< " init value: ";
		for(int i=0; i<preSampleWS.size(); i++)
			fout_WS << preSampleWS[i] << " " ;
		fout_WS << std::endl;

		int iterationNum = 0;
		int idxDiff = 0;
		do
		{
			std::vector<int> tmpSampleWS(m_materials->m_numCtrlPoint);
			tmpSampleWS = preSampleWS;

			std::map<double, int> covs;
			for(int i=0; i<m_materials->m_numCtrlPoint; i++)
			{
				double tmpCOV = WarmStartLineSearch(i, preSampleWS, freqDiff_LineSearch);
				// std::map use descene order to sort, so need minus in front
				covs.insert(std::pair<double, int>(-tmpCOV, i));
			}

			// output freq difference for all the line search point
			fout_WS<< " totalDiff: ";
			for(int i=0; i<freqDiff_LineSearch.size(); i++)
				fout_WS <<  freqDiff_LineSearch[i]  << " " ;
			fout_WS << std::endl;


			// output the covariance in order for each paramter
			// the bigger of the cov means the freq is more sensitive to the parameter
			std::map<double, int>::iterator iter;
			for(iter=covs.begin(); iter!=covs.end(); iter++)
				fout_WS << "cov: " << iter->second << " " << -1*iter->first << std::endl;

			///////////////////////
			//for(iter=covs.begin(); iter!=covs.end(); iter++)
			do
			{
				iter = covs.begin();
				//fout_WS << " current begin " << iter->second << " " <<iter->first << std::endl; 
				int tmpIdx = FindMinSample(iter->second*m_numSampleWS, freqDiff_LineSearch);
				fout_WS << " find minimu of " << iter->second << " at " << tmpIdx << std::endl; 
				if(tmpIdx != preSampleWS[iter->second])
					preSampleWS[iter->second] = tmpIdx + int((double)(preSampleWS[iter->second]-tmpIdx)/2.0);

				covs.erase(iter);

				if(covs.size()!=0)
				{
					iter = covs.begin();
					double tmpCOV = WarmStartLineSearch(iter->second, preSampleWS, freqDiff_LineSearch);
					for(iter=covs.begin(); iter!=covs.end(); iter++)
						fout_WS << "cov: " << iter->second << " " << -1*iter->first << std::endl;
				}


			/*	covs.erase(iter);
				std::map<double, int> tmp_covs;
				for(iter=covs.begin(); iter!=covs.end(); iter++)
				{
					double tmpCOV = WarmStartLineSearch(iter->second, preSampleWS, freqDiff_LineSearch);
					tmp_covs.insert(std::pair<double, int>(-tmpCOV, iter->second));
				}
				covs = tmp_covs;*/


				/*for(iter=covs.begin(); iter!=covs.end(); iter++)
					fout_WS << "cov: " << iter->second << " " << -1*iter->first << std::endl;*/

			}while(covs.size()!=0);
		
			fout_WS << iterationNum << " iteration result: ";
			for(int i=0; i<preSampleWS.size(); i++)
				fout_WS << preSampleWS[i] << " " ;
			fout_WS << std::endl;

			iterationNum++;

			idxDiff = 0;
			for(int i=0; i<	m_materials->m_numCtrlPoint; i++)
				idxDiff += abs(tmpSampleWS[i]-preSampleWS[i]);
			
		} while(idxDiff!=0);
	}

	double TrajParameterProblem::WarmStartLineSearch(int j, std::vector<int>& curSample, std::vector<double>& lineSearch)
	{
		//std::fill(m_WSSample.begin(), m_WSSample.end(), int((double)m_numSampleWS/2.0));
		m_WSSample = curSample;

		// set all the parameter defaultly at the middle of range
		std::vector<double> curValue(m_numOptVariable); 
		for(int i=0; i<m_materials->m_numCtrlPoint; i++)
			curValue[i] = m_lowBoundWS[i] + m_WSSample[i]*(m_upBoundWS[i]-m_lowBoundWS[i])/(double)m_numSampleWS;
		curValue[m_numOptVariable-2] = m_stiffnessDampWS;
		curValue[m_numOptVariable-1] = m_massDampWS;

		double mean = 0.0;
		for(int i=0; i<m_numSampleWS; i++)
		{
			curValue[j] = m_lowBoundWS[j] + i*(m_upBoundWS[j]-m_lowBoundWS[j])/(double)m_numSampleWS;
			m_WSSample[j] = i;

			// line search the jth variable		
			lineSearch[i + j*m_numSampleWS] = WarmStart(curValue);
			std::cout << "fill: " << i+j*m_numSampleWS << std::endl;
			mean += lineSearch[i + j*m_numSampleWS];
		}

		double cov = COV(j*m_numSampleWS, lineSearch, mean/(double)m_numSampleWS);

		return cov;
	}

	double TrajParameterProblem::COV(int startIdx, std::vector<double>& data, double mean)
	{
		double cov = 0.0;
		for(int i=0; i<m_numSampleWS; i++)
			cov += (data[i+startIdx]-mean)*(data[i+startIdx]-mean);
		return cov;
	}

	int TrajParameterProblem::FindMinSample(int startIdx, std::vector<double>& data)
	{
		double minValue = 1e6;
		int idx = -1;
		for(int i=0; i<m_numSampleWS; i++)
		{
			if(data[i+startIdx]<minValue)
			{
				idx = i;
				minValue = data[i+startIdx];
			}	
		}

		return idx;
	}

	double TrajParameterProblem::WarmStart( std::vector<double>& curValue )
	{
		//std::vector<double> curValue(m_numOptVariable); 
		//for(int i=0; i<m_materials->m_numCtrlPoint; i++)
		//	curValue[i] = m_lowBoundWS[i] + m_WSSample[i]*(m_upBoundWS[i]-m_lowBoundWS[i])/(double)m_numSampleWS;
		//curValue[m_numOptVariable-2] = m_stiffnessDampWS;
		//curValue[m_numOptVariable-1] = m_massDampWS;

		//m_candidateStaticExampleForce = m_staticExampleForce;

		// find the reference shape using the static frame
		// the reference shape is stored in m_refShape
		RetriveReferenceShape(curValue, m_staticShape->GetVertices(), m_refShape);
		//OutputReferenceShape(m_refShape);

		std::shared_ptr<TetMeshVertices> restShapeVertices = std::make_shared<TetMeshVertices>();
		restShapeVertices->GetVertices() = m_refShape;
		TetMesh restTetMesh(m_topology, restShapeVertices, m_materials);
		restTetMesh.ComputeVolumeMass();
		m_pIntegrator->GetModel()->InitializeUndeformedStiffnessMatrix(&restTetMesh);

		m_pIntegrator->ComputeEigenMode(m_numModes, m_NaturalFrequencies, m_EigenModes);

		// compute the displacement with reference shape
		// use the static shape approximate the reference shape
		ComputeDisplacement(m_refShape);

		for(int k=0; k<m_exampleShapesDyn.size(); k++)
		{
			std::vector<double> tmpCoord(m_numModes);
			ComputeReducedCoord( m_exampleDisplacement[k], tmpCoord);

			memcpy(&m_exampleShapeDynReduced[k*m_numModes], tmpCoord.data(), sizeof(double)*m_numModes);
		}

		double freq_diff = 0.0;
		for(int h=0; h<(m_frameSegDyn.size()-1); h++)
		{
			for(int i=0; i<m_materials->m_numCtrlPoint; i++)
				fout_WS << m_WSSample[i] << " ";

			fout_WS << h << " ";;

			// find the first several significant frames
			double dataFreq = FindCandidateFrames(m_frameSegDyn[h], m_frameSegDyn[h+1], m_exampleTimeInterval[h]);
			double eigFreq = (2*M_PI)*m_NaturalFrequencies[0];

			//freq_diff += (eigFreq-dataFreq)*(eigFreq-dataFreq);
			freq_diff += fabs(eigFreq-dataFreq);

			fout_WS << eigFreq << " " << dataFreq << std::endl;

			//for(int i=0; i<m_exampleShapeDynReduced.size(); i++)
			//	std::cout << m_exampleShapeDynReduced[i] << " " ;
			//std::cout << std::endl;
			
			// calculate the amplitude
			double minAmp = FindMin(m_frameSegDyn[h], m_frameSegDyn[h+1], m_exampleShapeDynReduced);
			double maxAmp = FindMax(m_frameSegDyn[h], m_frameSegDyn[h+1], m_exampleShapeDynReduced);
			double Amp = 0.5*(maxAmp-minAmp);
			freq_diff *= Amp;
		}

		return freq_diff;
	}


	void TrajParameterProblem::DrawObjFunction( std::vector<double>& _curValue )
	{
		double objValue = 0.0;

		for(int i=0; i<m_materials->m_numCtrlPoint; i++)
				fout_objfun << m_WSSample[i] << " ";

		if (!RetriveReferenceShape(_curValue, m_staticShape->GetVertices(), m_refShape))
		{
			fout_objfun << 100000 << std::endl;
			return;
		}

		objValue = MatchTrajectory(_curValue);
		fout_objfun << objValue << std::endl;
	}

	bool TrajParameterProblem::RetriveReferenceShape(std::vector<double>& curValue, std::vector<Vector3d>& _input, std::vector<Vector3d>& _output)
	{
		for(int k=0; k<GetMaterial()->m_numCtrlPoint; k++)
			m_E[k] = curValue[k];
		GetMaterial()->SetNonUniformMaterial(GetMaterial()->m_ctrlWeight, m_E, m_nu, GetMaterial()->m_numCtrlPoint, m_numElement);

		// reset to static position
		m_pIntegrator->SetState(_input);
		m_pIntegrator->SetDampingParameters(0.6, 0.0) ;

		// find the reference shape which satisfied the static equilibriim
		int curRestIter = 0;
		while(curRestIter < 200)
		{
			m_pIntegrator->ClearExternalForce();
			if(!m_pIntegrator->BackwardEulerStepRestShape(0.1))
			{
				std::cout << "*************wrong****************" << std::endl;
				return false;
			}
			curRestIter ++;
		}

		memcpy(_output.data(), m_pIntegrator->GetNodalPosition().data(), m_numNode*sizeof(Vector3d));

		return true;
	}


	double TrajParameterProblem::MatchTrajectory(std::vector<double>& curValue)
	{
		for(int i=0; i<m_frameError.size(); i++)
			m_frameError[i] = 0.0; 

		// set dampoing property
		double simu_damping = curValue[m_numOptVariable-2];
		double mass_damping = curValue[m_numOptVariable-1];

		//double simu_damping = curValue[m_numOptVariable-1];
		//double mass_damping = 0.00001;

		m_pIntegrator->SetDampingParameters(simu_damping,mass_damping);

		// update rest shape
		std::shared_ptr<TetMeshVertices> restShapeVertices = std::make_shared<TetMeshVertices>();
		restShapeVertices->GetVertices() = m_refShape;
		TetMesh restTetMesh(m_topology, restShapeVertices, m_materials);
		restTetMesh.ComputeVolumeMass();
		m_pIntegrator->GetModel()->InitializeUndeformedStiffnessMatrix(&restTetMesh);

		//ComputeModes();

		//OutputEigenMode(m_EigenModes);
		//OutputReferenceShape(m_refShape);

		// find the shape similarity of motion trajectory
		for(int i=0; i<(m_frameSegDyn.size()-1); i++)
		{
			int startIndex = m_frameSegDyn[i];

			// set the first frame
			m_pIntegrator->ClearExternalForce();
			m_pIntegrator->SetState(m_exampleShapesDyn[startIndex]->GetVertices());

			int iterationTimes = m_frameSegDyn[i+1] - startIndex;
			//printf("start index: %d %d\n", startIndex, iterationTimes);
			int curIteration = 1;

			while(curIteration<iterationTimes)
			{
				int iterNum = m_iterationNum[i];
				//if(curIteration == 1)
				//	iterNum = 5;
				for(int j=0; j</*m_iterationNum[i]*/iterNum; j++)
				{
					if(!m_pIntegrator->BackwardEulerStep(m_iterationSetp[i]))
						std::cout << "*******************************#############" << curIteration+startIndex <<std::endl;
				}

				// test if the first frame is aligned
				//ExportFrames(curIteration+startIndex, GetDynExampleShape(curIteration+startIndex)->GetVertices(), m_pIntegrator->GetNodalPosition());

				m_frameError[curIteration+startIndex] = ComputeShapeEnergy(GetDynExampleShape(curIteration+startIndex)->GetVertices(), 
					m_pIntegrator->GetNodalPosition(), (double)curIteration/(double)iterationTimes, i);

				/*			m_frameError[curIteration+startIndex] = ComputeShapeEnergyReduced( &m_exampleShapeDynReduced[(curIteration+startIndex)*m_numModes],
				m_pIntegrator->GetNodalPosition(), m_refShape, (double)curIteration/(double)iterationTimes, i);*/

				curIteration ++;
			}
		}

		// evaluate the objective funtion
		double shapeDifference = 0.0;
		for(int i=0; i<m_frameError.size(); i++)
			shapeDifference += m_frameError[i];

		return shapeDifference;
	}


	double TrajParameterProblem::ComputeShapeEnergy(const std::vector<Vector3d> &nodalPosition1, const std::vector<Vector3d> &nodalPosition2, double ratio, int trajIndex)
	{
		double shapeEnergy = 0.0;

		for(int i=0; i<m_numNode; i++)
		{	
			Vector3d diff = (nodalPosition1[i] - nodalPosition2[i]);
			shapeEnergy += diff.Length();
		}

		return shapeEnergy;
	}

	double TrajParameterProblem::ComputeShapeEnergyReduced(double* exampleCoord, const std::vector<Vector3d> &nodalPosition1, const std::vector<Vector3d> &ref, double ratio, int trajIndex)
	{
		double shapeEnergy = 0.0;

		std::vector<double> _coord(m_numModes);
		std::vector<Vector3d> _dis(m_numNode);
		std::transform(nodalPosition1.begin(), nodalPosition1.end(), ref.begin(), _dis.begin(), std::minus<Vector3d>());

		ComputeReducedCoord(_dis, _coord);

		for(int i=0; i<m_numModes; i++)
		{
			/*double tmp = (*(exampleCoord+i) - _coord[i]);
			if(abs(tmp)>1e-2)*/
			//	shapeEnergy += (*(exampleCoord+i) - _coord[i])*(*(exampleCoord+i) - _coord[i]);
			shapeEnergy += (*(exampleCoord+i) - _coord[i]);
		}

		return sqrt(shapeEnergy);
		//return shapeEnergy;
	}

	void TrajParameterProblem::ExportFrames(int index, const std::vector<Vector3d> &nodalPosition1, const std::vector<Vector3d> &nodalPosition2)
	{
		// example frame
		QString format = ".txt";

		QString example_fileName = "Compare/example_" + QString::number(index) + format;
		QString test_fileName  = "Compare/test_" + QString::number(index) + format;

		QString exampleVel_fileName = "Compare/exampleVel_" + QString::number(index) + format;
		QString testVel_fileName  = "Compare/testVel_" + QString::number(index) + format;


		QString example_filePath = QDir::toNativeSeparators(example_fileName);
		QString test_filePath = QDir::toNativeSeparators(test_fileName);

		QString exampleVel_filePath = QDir::toNativeSeparators(exampleVel_fileName);
		QString testVel_filePath = QDir::toNativeSeparators(testVel_fileName);

		if(nodalPosition1.size() != m_numNode)
			printf("****** %d  ******************************\n", nodalPosition1.size());
		if(nodalPosition2.size() != m_numNode)
			printf("****** %d  ******************************\n", nodalPosition2.size());

		// export example frame
		tofstream fout(example_filePath.utf16());
		fout.precision(18);
		for(int i=0; i<nodalPosition1.size(); i++)
			fout << i << TEXT(" ") << nodalPosition1[i].X() << TEXT(" ") << nodalPosition1[i].Y() << TEXT(" ") << nodalPosition1[i].Z() << std::endl;

		// export simulation frame
		tofstream fout2(test_filePath.utf16());
		fout2.precision(18);
		for(int i=0; i<nodalPosition2.size(); i++)
			fout2 << i << TEXT(" ") << nodalPosition2[i].X() << TEXT(" ") << nodalPosition2[i].Y() << TEXT(" ") << nodalPosition2[i].Z() << std::endl;

		return;
	}


	TrajParameterProblem::TrajParameterProblem(std::shared_ptr<TetMeshTopology> topology)
		:RestShapeProblem(topology)
	{
		m_curState = 0;
		m_maxIterationNum = 300;
		m_exampleShapesDyn.clear();
		m_exampleTimeInterval.clear();

		// fisrt trajectory start from first 0
		m_frameSegDyn.clear();
		m_frameSegDyn.push_back(0);

		m_bOutputOpti = false;
		m_bOutputObjFun = false;
		m_bWarmStart = false;
	}

	TrajParameterProblem::~TrajParameterProblem(void)
	{
		if(m_bOutputOpti)
			fout_opti.close();

		if(m_bOutputObjFun)
			fout_objfun.close();

		if(fout_WS)
			fout_WS.close();
	}

	void TrajParameterProblem::InitOptiOutput(const QString &filename)
	{
		QString score_filePath = QDir::toNativeSeparators(filename);

		fout_opti.open(score_filePath.utf16());
		fout_opti.precision(18);
		m_bOutputOpti = true;
	}

	void TrajParameterProblem::InitWSOutput(const QString &filename)
	{
		QString score_filePath = QDir::toNativeSeparators(filename);

		fout_WS.open(score_filePath.utf16());
		fout_WS.precision(18);
		m_bWarmStart = true;

		m_WSSample.resize(m_materials->m_numCtrlPoint);
		memset(m_WSSample.data(), 0, sizeof(double)*m_materials->m_numCtrlPoint);
	}

	void TrajParameterProblem::InitObjFunOutput(const QString &filename)
	{
		QString score_filePath = QDir::toNativeSeparators(filename);

		fout_objfun.open(score_filePath.utf16());
		fout_objfun.precision(18);
		m_bOutputObjFun = true;

		m_WSSample.resize(m_materials->m_numCtrlPoint);
		memset(m_WSSample.data(), 0, sizeof(double)*m_materials->m_numCtrlPoint);
	}

	bool TrajParameterProblem::Initialize(const Vector3d &gravity, const std::vector<double> &vertMass, const std::shared_ptr<TetMesh>& tetMesh)
	{
		// compute acceleratio and velocity for each frame
		std::tcout << m_exampleShapesDyn.size() << TEXT(" Dyn example shapes loaded!") << std::endl;

		m_refShape.resize(m_numNode);

		RestShapeProblem::Initialize(gravity, vertMass, tetMesh);

		InitializeNLopt();

		InitModes();

		m_iterationNum.resize(m_exampleTimeInterval.size());
		m_iterationSetp.resize(m_exampleTimeInterval.size());
		for(int i=0; i<m_exampleTimeInterval.size(); i++)
		{
			//m_iterationNum[i] = 10;
			m_iterationNum[i] = 1;
			m_iterationSetp[i] = m_exampleTimeInterval[i] / m_iterationNum[i];
		}

		m_frameError.resize(m_exampleShapesDyn.size());

		m_E.resize(m_materials->m_numCtrlPoint);
		m_nu.resize(m_materials->m_numCtrlPoint, 0.43);

		//ComputeExampleVelAccel(gravity, vertMass, m_exampleShapesDyn, m_exampleSpeeds, m_exampleAccel, m_exampleForce);

		return true;
	}


	bool TrajParameterProblem::LoadExample(const tstring &prefix, const tstring &suffix, int digitWidth, 
		int startidx, int step, int endidx, int numLineToSkip, double timestep)
	{        
		for (int i = startidx; i <= endidx; i += step)
		{
			tstringstream fn;
			fn << prefix;
			if (digitWidth)
				fn << std::setw(digitWidth) << std::setfill(TEXT('0'));
			fn << i << suffix;
			std::tcout << TEXT("Loading ") << fn.str() << TEXT("... ");

			std::shared_ptr<TetMesh> tetMesh = std::make_shared<TetMesh>(m_topology, std::shared_ptr<TetMeshVertices>(), m_materials);
			if (!tetMesh->LoadVertices(fn.str(), numLineToSkip))
				continue;
			if (m_numNode != (int)tetMesh->GetVertices().size())
			{
				std::tcout << TEXT("Wrong vertex count: ") << tetMesh->GetVertices().size() << TEXT(" ") << m_numNode << TEXT(" needed!") << std::endl;
				continue;
			}            

			m_exampleShapesDyn.push_back(tetMesh);
			std::tcout << TEXT("Done!") << std::endl;
		}

		m_frameSegDyn.push_back((int)m_exampleShapesDyn.size());
		m_exampleTimeInterval.push_back(timestep*(double)step);

		return true;
	}


	void TrajParameterProblem::ComputeExampleVelAccel( const Vector3d &gravity, const std::vector<double> &vertMass,
		const std::vector<std::shared_ptr<TetMesh>>& exampleShape,
		std::vector<std::vector<Vector3d>>& exampleSpeeds,
		std::vector<std::vector<Vector3d>>& exampleAccel,
		std::vector<std::vector<Vector3d>>& exampleForce)
	{
		int nShapes = exampleShape.size();

		exampleSpeeds.resize(nShapes, std::vector<Vector3d>(m_numNode, Vector3d::ZERO));
		exampleAccel.resize(nShapes, std::vector<Vector3d>(m_numNode, Vector3d::ZERO));
		exampleForce.resize(nShapes, std::vector<Vector3d>(m_numNode, Vector3d::ZERO));

		for(int i=0; i<(m_frameSegDyn.size()-1); i++)
		{
			double exampleTimeStep = m_exampleTimeInterval[i];
			for (int vi = 0; vi < m_numNode; ++vi)
			{
				for (size_t fi = (m_frameSegDyn[i]+1); fi < m_frameSegDyn[i+1]; ++fi)
					exampleSpeeds[fi][vi] = (exampleShape[fi]->GetVertices()[vi] - exampleShape[fi - 1]->GetVertices()[vi]) / exampleTimeStep;

				for (size_t fi = (m_frameSegDyn[i]+1); fi < (m_frameSegDyn[i+1]-1); ++fi)
				{
					exampleAccel[fi][vi] = (exampleSpeeds[fi + 1][vi] - exampleSpeeds[fi][vi]) / exampleTimeStep;
					exampleForce[fi][vi] = vertMass[vi] * (exampleAccel[fi][vi] - gravity);
				}
			}
		}
	}


	void TrajParameterProblem::InitializeNLopt()
	{
		//m_NLopt = nlopt::opt(nlopt::LD_SLSQP, m_numOptVariable);
		m_NLopt = nlopt::opt(nlopt::LN_NELDERMEAD, m_numOptVariable);

		/*nlopt::opt m_localLopt = nlopt::opt(nlopt::LN_SBPLX, 3);
		m_NLopt.set_local_optimizer(m_localLopt);*/

		m_NLopt.set_min_objective(callbackFunc, this);

		// for global optimization must set the upper and lower bound
		std::vector<double> upB;
		std::vector<double> lowB;
		for(int i=0; i<m_numOptVariable; i++)
		{
			upB.push_back(1.0);
			lowB.push_back(0.0);
		}
		m_NLopt.set_upper_bounds(upB);
		m_NLopt.set_lower_bounds(lowB);

		// set the stoping criterian
		m_NLopt.set_xtol_rel(1e-4);
		m_NLopt.set_maxeval(10000);
		m_NLopt.set_ftol_abs(1e-3);
	}


	void TrajParameterProblem::EncodeVariable(const std::vector<double>& input, std::vector<double>& output)
	{
		for(int i=0; i<m_numOptVariable; i++)
			output[i] = (input[i]-m_lowBound[i])/(m_upBound[i]-m_lowBound[i]);
	}

	void TrajParameterProblem::DecodeVariable(const std::vector<double>& input, std::vector<double>& output)
	{
		for(int i=0; i<m_numOptVariable; i++)
			output[i] = input[i] * (m_upBound[i]-m_lowBound[i]) + m_lowBound[i];
	}


	void TrajParameterProblem::UpdateNLopt()
	{
		if(m_bWarmStart)
		{
			//do{
			//	std::vector<double> curValue(m_numOptVariable); 
			//	for(int i=0; i<m_materials->m_numCtrlPoint; i++)
			//		curValue[i] = m_lowBoundWS[i] + m_WSSample[i]*(m_upBoundWS[i]-m_lowBoundWS[i])/(double)m_numSampleWS;
			//	curValue[m_numOptVariable-2] = m_stiffnessDampWS;
			//	curValue[m_numOptVariable-1] = m_massDampWS;

			//	WarmStart(curValue);
			//}while(WarmStartCount());

			clock_t startTime = clock();
			fout_WS << "start time: " << (double)startTime/1000.0 << std::endl;
			WarmStartSampleFast();
			clock_t endTime = clock();
			fout_WS << "end time: " << (double)endTime/1000.0 << std::endl;

			//std::vector<double> curValue(m_numOptVariable); 
			//curValue[0] = 4e6;
			////curValue[1] = 1e4;
			////curValue[2] = 100000;
			////curValue[3] = 1e6;
			//curValue[m_numOptVariable-2] = 0.001;
			//curValue[m_numOptVariable-1] = 0.001;
			//WarmStart(curValue);
		}
		else if(m_bOutputObjFun)
			do{
				std::vector<double> curValue(m_numOptVariable); 
				for(int i=0; i<m_materials->m_numCtrlPoint; i++)
					//curValue[i] = pow(10.0, 4.0+m_WSSample[i]*(7.0-4.0)/(double)50.0);
					curValue[i] = m_lowBoundWS[i] + m_WSSample[i]*(m_upBoundWS[i]-m_lowBoundWS[i])/(double)m_numSampleWS;
				curValue[m_numOptVariable-2] = m_stiffnessDampWS;
				curValue[m_numOptVariable-1] = m_massDampWS;

				DrawObjFunction(curValue);
			}while(WarmStartCount());
		else
		{
			clock_t startTime = clock();
			fout_opti << "start time: " << (double)startTime/1000.0 << std::endl;
			std::vector<double> initValue(m_numOptVariable);
			std::vector<double> initEncodeValue(m_numOptVariable);
			for(int i=0; i<m_materials->m_numCtrlPoint; i++)
			{
				int varIndex = i*m_materials->m_numCtrlPoint;
				initValue[i] = m_materials->m_ctrlE[i];
			}
			//initValue[m_numOptVariable-2] = 0.04;
			//initValue[m_numOptVariable-1] = 0.002;
			initValue[m_numOptVariable-2] = 0.04;
			initValue[m_numOptVariable-1] = 0.0075;

			//initValue[m_numOptVariable-1] = 0.018;

			EncodeVariable(initValue, initEncodeValue);

			double minf;
			nlopt::result result = m_NLopt.optimize(initEncodeValue, minf);


			std::vector<double> FinalValue(m_numOptVariable);
			DecodeVariable(initEncodeValue, FinalValue);

			printf("found minimum because %d at f(", result);
			for(int i=0; i<m_numOptVariable; i++)
				printf("%g,", FinalValue[i]);
			printf(") = %0.10g\n", minf);
			clock_t endTime = clock();
			fout_opti << "end time: " << (double)endTime/1000.0 << std::endl;
		}
	}

	void TrajParameterProblem::InitModes()
	{
		m_numModes = 1;

		m_EigenModes.clear();
		m_EigenModes.resize(m_numModes, std::vector<Vector3d>(m_numNode, Vector3d::ZERO));

		m_NaturalFrequencies.clear();
		m_NaturalFrequencies.resize(m_numModes);

		m_exampleShapeDynReduced.clear();
		m_exampleShapeDynReduced.resize(m_numModes*m_exampleShapesDyn.size());
	}

	void TrajParameterProblem::ComputeModes()
	{
		ComputeDisplacement(m_refShape);
		double* sketchData = new double[m_exampleDisplacement.size()*m_numNode*3];
		for(int i=0; i<m_exampleDisplacement.size(); i++)
		{
			int beginIdx = i*m_numNode*3;
			for(int j=0; j<m_numNode; j++)
			{
				sketchData[beginIdx+j*3+0] = m_exampleDisplacement[i][j].X();
				sketchData[beginIdx+j*3+1] = m_exampleDisplacement[i][j].Y();
				sketchData[beginIdx+j*3+2] = m_exampleDisplacement[i][j].Z();
			}
		}

		m_pIntegrator->ComputeEigenMode(m_numModes, m_exampleShapesDyn.size(), sketchData, m_NaturalFrequencies, m_EigenModes);

		delete[] sketchData;

		//m_pIntegrator->ComputeEigenMode(m_numModes, m_NaturalFrequencies, m_EigenModes);

		for(int j=0; j<m_exampleShapesDyn.size(); j++)
		{
			std::vector<double> tmpCoord(m_numModes);
			ComputeReducedCoord( m_exampleDisplacement[j], tmpCoord);

			memcpy(&m_exampleShapeDynReduced[j*m_numModes], tmpCoord.data(), sizeof(double)*m_numModes);
		}
	}


	void TrajParameterProblem::ComputeTrajactoryMode(int startIdx, int endIdx)
	{
		double* sketchData = new double[(endIdx-startIdx)*m_numNode*3];
		for(int i=startIdx; i<endIdx; i++)
		{
			int beginIdx = (i-startIdx)*m_numNode*3;
			for(int j=0; j<m_numNode; j++)
			{
				sketchData[beginIdx+j*3+0] = m_exampleDisplacement[i][j].X();
				sketchData[beginIdx+j*3+1] = m_exampleDisplacement[i][j].Y();
				sketchData[beginIdx+j*3+2] = m_exampleDisplacement[i][j].Z();
			}
		}
		m_pIntegrator->ComputeEigenMode(m_numModes, (endIdx-startIdx), sketchData, m_NaturalFrequencies, m_EigenModes);

		delete[] sketchData;
	}

	double TrajParameterProblem::FindCandidateFrames(int startIdx, int endIdx, double time)
	{
		std::vector<int> stationary_idx;
		// find the stationary point of the reduced coord and store as candidate
		stationary_idx.push_back(0);
		for(int j=startIdx+1; j<endIdx-1; j++)
		{
			double coord1 = m_exampleShapeDynReduced[(j-1)*m_numModes];
			double coord2 = m_exampleShapeDynReduced[(j+0)*m_numModes];
			double coord3 = m_exampleShapeDynReduced[(j+1)*m_numModes];

			double finite_diff = (coord2-coord1) * (coord3-coord2);
			if(finite_diff<0.0)
			{
				stationary_idx.push_back(j);
				std::cout << "stationary push back: " << j << " " << startIdx << std::endl;
			}
		}

		// calculate the frequency 
		std::cout << "stationary size :" << stationary_idx.size() << std::endl;
		//if(stationary_idx.size()>=2)
		//{
		//	double freq = (2 * M_PI) / (2.0*(double)(stationary_idx[stationary_idx.size()-2] - stationary_idx[stationary_idx.size()-3])*time);
		//	fout_WS << stationary_idx[stationary_idx.size()-2] << " " << stationary_idx[stationary_idx.size()-3] << " ";
		//	std::cout << "stationary point: " << stationary_idx[stationary_idx.size()-2] << " " << stationary_idx[stationary_idx.size()-3] << " " << freq << " " << m_NaturalFrequencies[0]<< std::endl;
		//	return freq;
		//}
		//else
		//	return 0;

		if(stationary_idx.size()>=3)
		{
			double freq = (2 * M_PI) / ((double)(stationary_idx[2] - stationary_idx[1])*time);
			fout_WS << stationary_idx[2] << " " << stationary_idx[1] << " ";
			std::cout << "stationary point: " << stationary_idx[stationary_idx.size()-2] << " " << stationary_idx[stationary_idx.size()-1] << " " << freq << " " << m_NaturalFrequencies[0]<< std::endl;
			return freq;
		}
		else
			return 100000;

	}


	void TrajParameterProblem::ComputeDisplacement(std::vector<Vector3d> _ref)
	{
		int numExample = m_exampleShapesDyn.size();
		m_exampleDisplacement.clear();
		m_exampleDisplacement.resize(numExample, std::vector<Vector3d>(m_numNode, Vector3d::ZERO));

		for(int i=0; i<numExample; i++)
		{
			for (int j=0; j<m_numNode; j++)
				m_exampleDisplacement[i][j] = m_exampleShapesDyn[i]->GetVertices()[j] - _ref[j];
		}
	}

	void TrajParameterProblem::ComputeReducedCoord(std::vector<Vector3d>& _disVec,  std::vector<double>& _coord)
	{
		for(int i=0; i<m_numModes; i++)
		{
			_coord[i] = 0.0;
			for(int j=0; j<_disVec.size(); j++)
				_coord[i] += _disVec[j].Dot(m_EigenModes[i][j]);
		}
	}


	void TrajParameterProblem::OutputEigenMode(std::vector<double>& _eigVec)
	{
		QString score_fileName = "EigenMode.txt";
		QString score_filePath = QDir::toNativeSeparators(score_fileName);
		tofstream fout;
		fout.open(score_filePath.utf16());
		fout.precision(18);

		for(int i=0; i<m_numModes; i++)
		{
			int startIdx = i*m_numNode*3;
			for(int j=0; j<m_numNode*3; j++)
				fout << _eigVec[startIdx+j] << TEXT(" ");
			fout<< std::endl;
		}

		fout.close();
	}

	void TrajParameterProblem::OutputReferenceShape(const std::vector<Vector3d>& input)
	{
		QString score_fileName = "referenceshapeSingle.txt";
		QString score_filePath = QDir::toNativeSeparators(score_fileName);
		tofstream fout;
		fout.open(score_filePath.utf16());
		fout.precision(18);

		for(int i=0; i<m_numNode; i++)
		{
			fout << input[i].X() << TEXT(" ") <<
				input[i].Y() << TEXT(" ") <<
				input[i].Z() << std::endl;
		}
		fout.close();
	}

	bool TrajParameterProblem::WarmStartCount()
	{
		bool carry = true;
		int curBit = 0;

		while(carry)
		{
			m_WSSample[curBit] += 1;
			if(m_WSSample[curBit]<m_numSampleWS)
				carry = false;
			else
			{
				if(curBit==(m_WSSample.size()-1))
					return false;

				m_WSSample[curBit] = 0;
				curBit += 1;
			}
		}

		return true;
	}

	double TrajParameterProblem::FindMin(int startIdx, int endIdx, std::vector<double>& data)
	{
		double cur_min = 1e6;
		for(int i=startIdx; i<endIdx; i++)
		{
			if(data[i]<cur_min)
				cur_min = data[i];
		}
		return cur_min;
	}

	double TrajParameterProblem::FindMax(int startIdx, int endIdx, std::vector<double>& data)
	{
		double cur_max = -1e6;
		for(int i=startIdx; i<endIdx; i++)
		{
			if(data[i]>cur_max)
				cur_max = data[i];
		}
		return cur_max;
	}

}