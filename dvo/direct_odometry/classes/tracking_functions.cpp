//
// Created by font_al on 10/18/18.
//

#include "trackingFrontEnd.h"
#include "../utils/optimizer.h"



void fv::TrackingFrontEnd::trackFrameDir() {

    initOptimizationVariables();


    //#ifdef g2o
/**/
    for (int iPyr{(trackedFrame->cam)->pyrLevelsUsed - 1}; iPyr >= 0; --iPyr) {
        ePhPrevious = ePhMean;
        setPyrLevelOptimization(iPyr);
        g2oOpimizer(iPyr);


        //delta_cw_ref = delta_cw;
        if(debugTracking){
            #ifdef VISUALIZATION
                trackedFrame->draw_points_and_Jacobians(hgpForTracking,J, iPyr, 6, trackedFrame->cam->mask_size);
            #ifndef ROS
                trackedFrame->show_image( "hgp");
            #endif
            #endif
            }

        ePhRelative = ePhMean / ePhPrevious;
//        if ((ePhRelative > 0.99)&&(ePhRelative < 1.01)||ePhMean<1e-15) { // Optimization Level Finished
//            break;
//        }

    }
    setOptimizationVariables();

    //#endif

    #ifdef naive
    for (int iPyr{(trackedFrame->cam)->pyrLevelsUsed - 1}; iPyr >= 0; --iPyr) {

        setPyrLevelOptimization(iPyr);

        computePhotometricError();
        for (size_t iteration{0}; iteration < 30; ++iteration) {
            ePhPrevious = ePhMean;
            computePhotometricJacobian();

            if(debugTracking){
                #ifdef VISUALIZATION
                trackedFrame->draw_points_and_Jacobians(hgpForTracking,J, iPyr, 6, trackedFrame->cam->mask_size);
                #ifndef ROS
                trackedFrame->show_image( "hgp");
                #endif
                #endif
            }

            solveOptimizationProblem();
            if(not satisfiesCinematicModelConditions(delta)) break;
            updateOptimizationVariables();
            computePhotometricError();
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Finish PyrLevel Optimization Step
            ePhRelative = ePhMean / ePhPrevious;
            if ((ePhRelative > 0.99)&&(ePhRelative < 1.01)&&(iteration > 2)) { // Optimization Level Finished
                break;
            }
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Check for pyramid level success
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }
    #endif

    trackFrameDirSucceed();


}

void fv::TrackingFrontEnd::g2oOpimizer(int iPyr) {
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<8, 1>> DirectBlock;  // 8*1 vector to be solved
    DirectBlock::LinearSolverType *linearSolver = new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>();
    DirectBlock *solver_ptr = new DirectBlock(linearSolver);
    //auto *solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
    auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); // L-M
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);
    //optimizer.optimize(30);

    // add vertex
    auto camwithSSD = new VertexCamerawithSSD();
    camwithSSD->setEstimate(delta_cw);
    camwithSSD->setId(0);
    optimizer.addVertex(camwithSSD);

    int id = 1;
    DATA_TYPE measurement = 0;

    //std::cout << "starting optimization++" << std::endl;

    std::vector<EdgeDirectwithSSD*> edges;
    edges.reserve(numHgpForTracking);
    setPyrLevelOptimization(iPyr);
    for (fv::Pt *pt:hgpForTracking) {
        for (size_t iMask = 0; iMask < trackedFrame->cam->mask_size; iMask++) {

            measurement = pt->I_ref[iMask][iPyr];
            auto edge = new EdgeDirectwithSSD(trackedFrame, pt, iMask,delta_cw_ref);
            edge->setVertex(0, camwithSSD);
            edge->setMeasurement(measurement);
            edge->setId(id++);
            edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
            //edge->setRobustKernel(new g2o::RobustKernelCauchy);
            optimizer.addEdge(edge);
            edges.push_back(edge);

//            for(size_t iMask{}; iMask < trackedFrame->cam->mask_size; ++iMask){
//                DATA_TYPE measurement = pt->I_ref[iMask][trackedFrame->cam->iPyr];
        }
    }
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    DATA_TYPE error{0};
    size_t nums=numHgpForTracking;
    for(auto i_edge:edges)
    {
        if(i_edge->chi2()>7.31)
        {
            i_edge->setLevel(1);
            nums--;
        }
        else{
            error+=SQRT(i_edge->chi2());

        }
        i_edge->setRobustKernel(0);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);
    ePhMean = error/nums;
    //std::cout << "the mean error is: " << ePhMean<<std::endl;
    delta_cw = camwithSSD->estimate();
}

void fv::TrackingFrontEnd::initOptimizationVariables(){

    trackingSucceed = false;

    // Save previous values
    std::copy_n(t_cw_track,3,t_cw_track_pre);
    std::copy_n(R_cw_track,9,R_cw_track_pre);
    v_cw_track_pre = v_cw_track;
    w_cw_track_pre = w_cw_track;

    delta_cw = EIGEN_VECTOR_X::Zero(8);
    delta_cw.head(3) = v_cw_track;
    delta_cw.head(6).tail(3) = w_cw_track;
    delta_cw(6) = phScalar_track;
    delta_cw(7) = phBias_track;

    delta_cw_ref = EIGEN_VECTOR_X::Zero(8);
    delta_cw_ref.head(3) = trackedFrame->v_cw;
    delta_cw_ref.head(6).tail(3) = trackedFrame->w_cw;
    delta_cw_ref(6) = phScalar_track;
    delta_cw_ref(7) = phBias_track;

}



void fv::TrackingFrontEnd::updateOptimizationVariables(){

    fv::exp_lie(delta_t, delta_R, delta.head(3), delta.head(6).tail(3));
    update_T_relative(t_cw_track, R_cw_track, delta_t, delta_R);
    fv::log_lie(v_cw_track,w_cw_track,t_cw_track,R_cw_track);

    phScalar_track += delta(6);
    phBias_track   += delta(7);

    delta_cw.head(3) = v_cw_track;
    delta_cw.head(6).tail(3) = w_cw_track;
    delta_cw(6) = phScalar_track;
    delta_cw(7) = phBias_track;

}

void fv::TrackingFrontEnd::getOptimizeraztionVariablesDif()
{

}

void fv::TrackingFrontEnd::setOptimizationVariables()
{

    v_cw_track = delta_cw.head(3);
    w_cw_track = delta_cw.head(6).tail(3);
    fv::exp_lie(t_cw_track, R_cw_track, v_cw_track, w_cw_track);
    phScalar_track = delta_cw(6);
    phBias_track = delta_cw(7);

}

void fv::TrackingFrontEnd::solveOptimizationProblem() {
    // A = J.transpose()* J;   b = -e.transpose()*J;

    A +=  accelerationMaxTrack ;
    b +=  accelerationMaxTrack*(delta_cw_ref-delta_cw);

    //delta = A.colPivHouseholderQr().solve(b);

    Eigen::EigenSolver<EIGEN_MATRIX_X> es(A);
    D = es.pseudoEigenvalueMatrix();
    V = es.pseudoEigenvectors();

    delta = V*D.inverse()*V.transpose()*b;

}

void fv::TrackingFrontEnd::setPyrLevelOptimization(const int iPyr){
    trackedFrame->cam->setResolution(iPyr);
    trackedFrame->cam->mask_size = 1;
    if (iPyr == 0)
        trackedFrame->cam->mask_size = MASK_SIZE;

    e = EIGEN_VECTOR_X::Zero(numHgpForTracking*trackedFrame->cam->mask_size);
    eAbs = EIGEN_VECTOR_X::Zero(numHgpForTracking*trackedFrame->cam->mask_size);
#ifdef VISUALIZATION
    J = EIGEN_MATRIX_X::Zero(numHgpForTracking*trackedFrame->cam->mask_size,8);
#endif


}

bool fv::TrackingFrontEnd::trackFrameDirSucceed(){


    /*if(not satisfiesCinematicModelConditions(deltaInc)){
        trackingSucceed = false;
        throw cinematicModelFail;
    }*/


    if(((ePhMean/lastPhError < maxRelativePhotoError)&&(ePhMean/lastPhError > minRelativePhotoError))||(not isFrontEndInitialized)||ePhMean<1e-15) {

        #ifdef CINEMATIC_MODEL_BLOCK_DEBUG
        *cinematicModelBlockLog << std::setprecision(14)<<
            (v_cw_track-v_cw_track_pre).norm()    << " "<<
            (v_cw_track-trackedFrame->v_cw).norm()<< " "<<
            (w_cw_track-w_cw_track_pre).norm()    << " "<<
            (w_cw_track-trackedFrame->w_cw).norm()<< " "<<
            "\n";
        #endif

        lastPhError = ePhMean;

        cinematicModelConditions();
        trackedFrame->set_T_cw(t_cw_track, R_cw_track);

        #ifdef ENTROPY
        //computeEntropy(A);
        #endif

        #ifdef VISUALIZATION
        trackedFrame->draw_points_and_Jacobians(hgpForTracking,J, 0, 6, trackedFrame->cam->mask_size);
            #ifndef ROS
            trackedFrame->show_image( "hgp");
            #endif
        #endif

        trackedFrame->phScalar = phScalar_track;
        trackedFrame->phBias   = phBias_track;

        trackingSucceed = true;
    }
    else{

        #ifdef VISUALIZATION
        trackedFrame->draw_points_and_Jacobians(hgpForTracking,J,0,6, trackedFrame->cam->mask_size);
            #ifndef ROS
            trackedFrame->show_image( "hgp");
            #endif
        #endif

        std::cout << "TRACKING FAILED (Photometric Error Threshold) "<< std::endl;
        std::cout << "ePhMean = " << ePhMean << std::endl;
        std::cout << "lastPhError = " << lastPhError << std::endl;
        std::cout << "ePhMean/lastPhError = " << ePhMean/lastPhError << std::endl;
        std::cout << "phScalar_track[0] = " << phScalar_track << std::endl;
        std::cout << "trackedFrame->phScalar = " << trackedFrame->phScalar << std::endl;
        std::cout << "phScalar_track[0]/trackedFrame->phScalar[0] = " << phScalar_track/trackedFrame->phScalar<< std::endl;

        trackingSucceed = false;

        //linearVelocity  = EIGEN_VECTOR_3::Zero(3);
        //angularVelocity = EIGEN_VECTOR_3::Zero(3);

        throw photometricErrorFail;
    }
}

bool fv::TrackingFrontEnd::cinematicModelConditions(){

    linearVelocities.emplace_back((v_cw_track - trackedFrame->v_cw)/inc_t1);
    angularVelocities.emplace_back((w_cw_track - trackedFrame->w_cw)/inc_t1);

    size_t numVelocities = linearVelocities.size();
    if(numVelocities > sysSet.numTrackedFramesToInitCinematicModel){
        linearVelocities.erase(linearVelocities.begin());
        angularVelocities.erase(angularVelocities.begin());
        --numVelocities;
    }

    linearVelocity = EIGEN_VECTOR_3::Zero(3);
    angularVelocity = EIGEN_VECTOR_3::Zero(3);
    for(size_t iVelocity{}; iVelocity < numVelocities; ++iVelocity) {
        linearVelocity  += linearVelocities[iVelocity];
        angularVelocity += angularVelocities[iVelocity];
    }
    linearVelocity  /= numVelocities;
    angularVelocity /= numVelocities;

    return true;
}

bool fv::TrackingFrontEnd::satisfiesCinematicModelConditions(EIGEN_VECTOR_X& delta_){

    if (not delta_.array().isFinite().sum() ){
        std::cout << "TRACKING FAILED (Infinite Delta) "<< std::endl;
        //std::cout << "J = " << J << std::endl;
        //std::cout << "e = " << e << std::endl;
        std::cout << "b = " << b << std::endl;
        std::cout << "A = " << A << std::endl;
        std::cout << "delta = " << delta << std::endl;
        return false;
    }

    /*if ((delta_.head(6).tail(3).array().abs() > 0.05).sum() ){
        std::cout << "TRACKING FAILED (Angular Velocity Threshold) "<< std::endl;
        std::cout << "delta = " << delta_.head(6).tail(3).array().abs()  << std::endl;
        return false;
    }

    if ((delta_.head(3).array().abs() > 0.05).sum() ){
        std::cout << "TRACKING FAILED (Linear Velocity Threshold) "<< std::endl;
        std::cout << "delta = " << delta_.head(3).array().abs()  << std::endl;
        return false;
    }*/



    return true;
}


void fv::TrackingFrontEnd::computePhotometricError() {

    size_t indexPt{0};

    int maskSize = int(trackedFrame->cam->mask_size);
    EIGEN_VECTOR_X ei{EIGEN_VECTOR_X::Zero(maskSize)};

    for (fv::Pt* pt: hgpForTracking) {

        trackedFrame->XYZ_2_xynlambda(*pt,t_cw_track,R_cw_track);
        trackedFrame->cam->xyn_2_uv(*pt);

        trackedFrame->computePhotometricError(ei,*pt, phScalar_track,phBias_track);

        e.segment(indexPt*maskSize,maskSize) = ei;
        eAbs.segment(indexPt*maskSize,maskSize) = ei.array().abs();
        ++indexPt;

    }
    photometricErrorDistribution();
}

void fv::TrackingFrontEnd::photometricErrorDistribution(){
    ePhMean = eAbs.mean();
    DATA_TYPE ePhStd{SQRT(((eAbs.array()-ePhMean).square().sum())/(eAbs.size()-1))};
    HuberPh = ePhMean+2*ePhStd;
    ZeroPh = ePhMean+3*ePhStd;

    for(size_t ei{}; ei < e.size(); ++ei){
        if (eAbs(ei)  > ZeroPh){
            e(ei) = 1.4141*(e(ei)/eAbs(ei))*ZeroPh;
        }
        else{
            if(eAbs(ei)  > HuberPh){
                e(ei) = (e(ei)/eAbs(ei))*SQRT(2*HuberPh*(eAbs(ei) -0.5*HuberPh));
            }
        }
    }
}

void fv::TrackingFrontEnd::computePhotometricJacobian() {

    size_t indexPt{0};
    int maskSize = int(trackedFrame->cam->mask_size);

    EIGEN_MATRIX_X Ji = EIGEN_MATRIX_X::Zero(maskSize,8);
    EIGEN_VECTOR_X eAbsi;
    EIGEN_VECTOR_X ei;

    A = EIGEN_MATRIX_X::Zero(8,8);
    b = EIGEN_VECTOR_X::Zero(8);

    for (fv::Pt* pt: hgpForTracking) {
        eAbsi = eAbs.segment(indexPt*maskSize,maskSize);
        ei = e.segment(indexPt*maskSize,maskSize);
        trackedFrame->computePhotometricJacobian(*pt,Ji,ei,eAbsi, ZeroPh, HuberPh);

        //e.segment(indexPt*maskSize,maskSize) /= (pt->stdLambdaRef);

        A += Ji.transpose()*Ji;
        b -= ei.transpose()*Ji;

        #ifdef VISUALIZATION
        J.block(indexPt*maskSize,0,maskSize,8)  =  Ji;
        pt->Jrow = indexPt*maskSize;
        #endif

        ++indexPt;
    }
}


void fv::TrackingFrontEnd::update_T_relative(VECTOR3& t_relative, MATRIX3& R_relative, VECTOR3 delta_t_cw,MATRIX3 delta_R_cw) {
    // R_relative = delta_R_cw * R_relative;
    // t_relative = delta_R_cw * t_relative + delta_t_cw

    MATRIX3 R_cw_temp{};
    VECTOR3 t_cw_temp{};

    fv::Matrix3_x_Matrix3(R_cw_temp,delta_R_cw,R_relative);
    fv::rotate_translate_v(t_cw_temp,t_relative,delta_R_cw,delta_t_cw);

    fv::assign_Matrix3(R_relative,R_cw_temp);
    fv::assign_Vector3(t_relative,t_cw_temp);
}



void fv::TrackingFrontEnd::resetTracking(){

    std::cout << "RESET TRACKING"<< std::endl;

    // Save previous values
    std::copy_n(t_cw_track_pre,3,t_cw_track);
    std::copy_n(R_cw_track_pre,9,R_cw_track);
    v_cw_track = v_cw_track_pre;
    w_cw_track = w_cw_track_pre;

    phScalar_track = trackedFrame->phScalar;
    phBias_track = trackedFrame->phBias;

    trackedFrame->cam->mask_size = MASK_SIZE;

    //fv::waitFunction("YES");
}

#ifdef ENTROPY
void fv::TrackingFrontEnd::computeEntropy(const EIGEN_MATRIX_X& COV, std::vector <size_t > iDOF_Vector) {

    EIGEN_MATRIX_X COV_aux;
    EIGEN_MATRIX_X COV_aux1;
    EIGEN_MATRIX_X Dblock;

    //trackedFrame->globalEntropy = LOGARITHM(1/COV.determinant());
    //std::cout <<  " trackedFrame->globalEntropy = "<<trackedFrame->globalEntropy << std::endl;

    for(size_t iDOF: iDOF_Vector){
        COV_aux  = COV;
        COV_aux1 = COV;
        COV_aux.row(iDOF) = COV_aux1.row(0);
        COV_aux.row(0) = COV_aux1.row(iDOF);
        COV_aux1 = COV_aux;
        COV_aux.col(iDOF) = COV_aux1.col(0);
        COV_aux.col(0) = COV_aux1.col(iDOF);
        Dblock = COV_aux.block<5,5>(1,1);
        trackedFrame->entropy[iDOF] = LOGARITHM(Dblock.determinant()/COV.determinant());
    }

}
#endif