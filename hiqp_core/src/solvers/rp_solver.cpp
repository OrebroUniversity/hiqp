#include <hiqp/solvers/rp_solver.h>


namespace hiqp {

RPSolver::RPSolver() {

}

bool RPSolver::solve(std::vector<double>& solution) {

  if (stages_map_.empty()) return false;

  n_solution_dims_ = solution.size();
  unsigned int current_priority = 0;

  //solution at current and previous priority levels
  Eigen::MatrixXd ddq_(n_solution_dims_,1), ddq_prev_(n_solution_dims_,1);
  //reverse augmented jacobian J_RA, pseudoinverse version Tk
  Eigen::MatrixXd T_k, J_RA;

  //set previous solution to 0. should be -\lambda ddq_{i-1}
  ddq_prev_.setZero();

  for (auto kv=stages_map_.rbegin(); kv!=stages_map_.rend(); kv++) {
    current_priority = kv->first;

    const HiQPStage& current_stage = kv->second;

    //DEBUG =========================================
    std::cerr<<"Current stage: "<<std::endl;
    std::cerr<<"stage priority: "<<current_priority<<std::endl;
    std::cerr<<"B_= [\n"<<std::endl<<current_stage.B_<<"];\n";
    //std::cerr<<"constraint_signs_: ";
    //for (int i=0; i<current_stage.constraint_signs_.size();i++)
    //  std::cerr<<current_stage.constraint_signs_[i]<<" ";

    //std::cerr<<std::endl;
    std::cerr<<"b_= [ "<<std::endl<<current_stage.b_<<"];\n";
    //END DEBUG =========================================

    //first priority, initialize the reverse jacobian
    if(J_RA.rows()==0) {
    	/*
	for (int i=0; i<current_stage.constraint_signs_.size();i++) {
	  if(current_stage.constraint_signs_[i]==0) {

	  } else {
	    //FIXME inequality constraints
	  }
	}*/
	J_RA=current_stage.B_;
        T_k =pseudoInverse(J_RA);	
    } else {
	Eigen::MatrixXd J_RA_k (J_RA.rows()+current_stage.B_.rows(),n_solution_dims_);
	J_RA_k.topRows(current_stage.B_.rows()) = current_stage.B_;	
	J_RA_k.bottomRows(J_RA.rows()) = J_RA;
	J_RA = J_RA_k;
        Eigen::MatrixXd J_RA_pseudoinv = pseudoInverse(J_RA);	
	T_k = Eigen::Block<Eigen::MatrixXd>(J_RA_pseudoinv,0,0, n_solution_dims_, current_stage.B_.rows()); 	
    }
    
    Eigen::MatrixXd JTK = current_stage.B_*T_k;    
    std::cerr<<"J_RA = [\n"<<J_RA<<"];\n";
    std::cerr<<"T_k = [\n"<<T_k<<"];\n";
    std::cerr<<"JTK =[\n"<<JTK<<"];\n";

    ddq_ = ddq_prev_ + T_k*pseudoInverse(JTK)*(current_stage.b_ - current_stage.B_*ddq_prev_);
    ddq_prev_ = ddq_;
    std::cerr<<"ddq = ["<<std::endl<<ddq_.transpose()<<"]';\n";
  }

  solution = std::vector<double>(ddq_.data(), ddq_.data() + ddq_.rows() * ddq_.cols()); 
  return true;
}

}
