//
// Created by dair on 8/1/25.
//
LCS LCSFactory::LinearizePlantToLCS(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    const MultibodyPlant<AutoDiffXd>& plant_ad,
    const Context<AutoDiffXd>& context_ad,
    const vector<SortedPair<GeometryId>>& contact_geoms,
    int num_friction_directions, const std::vector<double>& mu, double dt,
    int N, ContactModel contact_model) {


  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;

  std::string base_path = "/home/dair/opt/test/new_delet_z_sampling_c3/dairlib/examples/sampling_c3/push_t/";
  SamplingC3Options sampling_c3_options =
    drake::yaml::LoadYamlFile<SamplingC3Options>(
        base_path + "parameters/sampling_c3_options.yaml");



  int n_x = plant_ad.num_positions() + plant_ad.num_velocities();
  int n_u = plant_ad.num_actuators();


  int n_contacts = contact_geoms.size();

  DRAKE_DEMAND(plant_ad.num_velocities() == plant.num_velocities());
  DRAKE_DEMAND(plant_ad.num_positions() == plant.num_positions());
  DRAKE_DEMAND(mu.size() == n_contacts);
  int n_v = plant.num_velocities();
  int n_q = plant.num_positions();

  AutoDiffVecXd C(n_v);
  plant_ad.CalcBiasTerm(context_ad, &C);
  VectorXd u_dyn = plant.get_actuation_input_port().Eval(context);

  auto B_dyn_ad = plant_ad.MakeActuationMatrix();
  AutoDiffVecXd Bu =
      B_dyn_ad * plant_ad.get_actuation_input_port().Eval(context_ad);

  AutoDiffVecXd tau_g = plant_ad.CalcGravityGeneralizedForces(context_ad);

  drake::multibody::MultibodyForces<AutoDiffXd> f_app(plant_ad);
  plant_ad.CalcForceElementsContribution(context_ad, &f_app);

  MatrixX<AutoDiffXd> M(n_v, n_v);
  plant_ad.CalcMassMatrix(context_ad, &M);

  // If this ldlt is slow, there are alternate formulations which avoid it
  AutoDiffVecXd vdot_no_contact =
      M.ldlt().solve(tau_g + Bu + f_app.generalized_forces() - C);
  // Constant term in dynamics, d_vv = d + A x_0 + B u_0
  VectorXd d_vv = ExtractValue(vdot_no_contact);
  // Derivatives w.r.t. x and u, AB
  MatrixXd AB_v = ExtractGradient(vdot_no_contact);
  VectorXd x_dvv(n_q + n_v + n_u);
  x_dvv << plant.GetPositions(context), plant.GetVelocities(context), u_dyn;
  VectorXd x_dvvcomp = AB_v * x_dvv;
  VectorXd d_v = d_vv - x_dvvcomp;

  ///////////
  AutoDiffVecXd x_ad = plant_ad.GetPositionsAndVelocities(context_ad);
  AutoDiffVecXd qdot_no_contact(n_q);
  AutoDiffVecXd vel_ad = x_ad.tail(n_v);

  plant_ad.MapVelocityToQDot(context_ad, vel_ad, &qdot_no_contact);
  MatrixXd AB_q = ExtractGradient(qdot_no_contact);
  MatrixXd d_q = ExtractValue(qdot_no_contact);
  Eigen::SparseMatrix<double> Nqt;
  Nqt = plant.MakeVelocityToQDotMap(context);
  MatrixXd qdotNv = MatrixXd(Nqt);

  Eigen::SparseMatrix<double> NqI;
  NqI = plant.MakeQDotToVelocityMap(context);
  MatrixXd vNqdot = MatrixXd(NqI);

  VectorXd phi(n_contacts);
  MatrixXd J_n(n_contacts, n_v);
  MatrixXd J_t(2 * n_contacts * num_friction_directions, n_v);

  if (!sampling_c3_options.with_z) {
    J_t.resize(2 * (n_contacts-1) * num_friction_directions + num_friction_directions, n_v);

  }

  for (int i = 0; i < n_contacts; i++) {
    multibody::GeomGeomCollider collider(
        plant,
        contact_geoms[i]);
    if (num_friction_directions == 1 || (!(sampling_c3_options.with_z) && i ==0)) {
      Eigen::Vector3d planar_normal;
      int num_direction = 1;

      planar_normal << 0, 1, 0;
      auto [phi_i, J_i] = collider.EvalPlanar(context, planar_normal);
      phi(i) = phi_i;
      J_n.row(i) = J_i.row(0);
      J_t.block(2 * i * num_direction, 0, 2 * num_direction,
                n_v) = J_i.block(1, 0, 2 * num_direction, n_v);
    } else {
      auto [phi_i, J_i] =
          collider.EvalPolytope(context, num_friction_directions);
      phi(i) = phi_i;
      J_n.row(i) = J_i.row(0);
      if (sampling_c3_options.with_z){
        J_t.block(2 * i * num_friction_directions, 0, 2 * num_friction_directions,
                  n_v) = J_i.block(1, 0, 2 * num_friction_directions, n_v);
      }else {
        J_t.block(2 * i * num_friction_directions-2, 0, 2 * num_friction_directions,
          n_v) = J_i.block(1, 0, 2 * num_friction_directions, n_v);


      }
    }

    // J_i is 3 x n_v
    // row (0) is contact normal
    // rows (1-num_friction directions) are the contact tangents
  }

  auto M_ldlt = ExtractValue(M).ldlt();
  MatrixXd MinvJ_n_T = M_ldlt.solve(J_n.transpose());
  MatrixXd MinvJ_t_T = M_ldlt.solve(J_t.transpose());

  MatrixXd A = MatrixXd::Zero(n_x, n_x);
  MatrixXd B = MatrixXd::Zero(n_x, n_u);
  VectorXd d = VectorXd::Zero(n_x);

  MatrixXd AB_v_q = AB_v.block(0, 0, n_v, n_q);
  MatrixXd AB_v_v = AB_v.block(0, n_q, n_v, n_v);
  MatrixXd AB_v_u = AB_v.block(0, n_x, n_v, n_u);
  MatrixXd M_double = MatrixXd::Zero(n_v, n_v);
  plant.CalcMassMatrix(context, &M_double);

  A.block(0, 0, n_q, n_q) =
      MatrixXd::Identity(n_q, n_q) + dt * dt * qdotNv * AB_v_q;
  A.block(0, n_q, n_q, n_v) = dt * qdotNv + dt * dt * qdotNv * AB_v_v;
  A.block(n_q, 0, n_v, n_q) = dt * AB_v_q;
  A.block(n_q, n_q, n_v, n_v) = dt * AB_v_v + MatrixXd::Identity(n_v, n_v);

  B.block(0, 0, n_q, n_u) = dt * dt * qdotNv * AB_v_u;
  B.block(n_q, 0, n_v, n_u) = dt * AB_v_u;

  d.head(n_q) = dt * dt * qdotNv * d_v;
  d.tail(n_v) = dt * d_v;

  MatrixXd E_t =
      MatrixXd::Zero(n_contacts, 2 * n_contacts * num_friction_directions);


  if (!sampling_c3_options.with_z) {
    E_t.resize(n_contacts, 2 * n_contacts * num_friction_directions - 2);
  }

  for (int i = 0; i < n_contacts; i++) {

    if (sampling_c3_options.with_z) {
      E_t.block(i, i * (2 * num_friction_directions), 1,
                2 * num_friction_directions) =
          MatrixXd::Ones(1, 2 * num_friction_directions);

    }else {
      if (i == 0) {
        int temp_num_friction_directions = 1;

        E_t.block(i, i * (2 * temp_num_friction_directions), 1,
          2 * temp_num_friction_directions) =
        MatrixXd::Ones(1, 2 * temp_num_friction_directions);
      }else {
        E_t.block(i, i * (2 * num_friction_directions)-2, 1,
          2 * num_friction_directions) =
        MatrixXd::Ones(1, 2 * num_friction_directions);

      }

    }


  }

  int n_lambda = 0;
  if (contact_model == ContactModel::kStewartAndTrinkle) {
    n_lambda = 2 * n_contacts + 2 * n_contacts * num_friction_directions;
  } else {
    if (sampling_c3_options.with_z) {
      n_lambda = 2 * n_contacts * num_friction_directions;
    }else {
      n_lambda = 2 * n_contacts * num_friction_directions -2;
    }

  }


  std::cerr << "lambda in lcs factory" << n_lambda << std::endl;

  std::cerr << "n_contacts" << n_contacts << std::endl;

  std::cerr << "num_friction_directions" << num_friction_directions << std::endl;



  // Matrices with contact variables
  MatrixXd D = MatrixXd::Zero(n_x, n_lambda);
  MatrixXd E = MatrixXd::Zero(n_lambda, n_x);
  MatrixXd F = MatrixXd::Zero(n_lambda, n_lambda);
  MatrixXd H = MatrixXd::Zero(n_lambda, n_u);
  VectorXd c = VectorXd::Zero(n_lambda);

  MatrixXd W_x = MatrixXd::Zero(n_lambda, n_x);
  MatrixXd W_l = MatrixXd::Zero(n_lambda, n_lambda);
  MatrixXd W_u = MatrixXd::Zero(n_lambda, n_u);
  MatrixXd w = VectorXd::Zero(n_lambda);

  if (contact_model == ContactModel::kStewartAndTrinkle) {
    D.block(0, 2 * n_contacts, n_q, 2 * n_contacts * num_friction_directions) =
        dt * dt * qdotNv * MinvJ_t_T;
    D.block(n_q, 2 * n_contacts, n_v,
            2 * n_contacts * num_friction_directions) = dt * MinvJ_t_T;
    D.block(0, n_contacts, n_q, n_contacts) = dt * dt * qdotNv * MinvJ_n_T;

    D.block(n_q, n_contacts, n_v, n_contacts) = dt * MinvJ_n_T;
    // Complementarity condition for gamma: mu lambda^n
    E.block(n_contacts, 0, n_contacts, n_q) =
        dt * dt * J_n * AB_v_q + J_n * vNqdot;
    E.block(2 * n_contacts, 0, 2 * n_contacts * num_friction_directions, n_q) =
        dt * J_t * AB_v_q;
    E.block(n_contacts, n_q, n_contacts, n_v) =
        dt * J_n + dt * dt * J_n * AB_v_v;
    E.block(2 * n_contacts, n_q, 2 * n_contacts * num_friction_directions,
            n_v) = J_t + dt * J_t * AB_v_v;

    VectorXd mu_vec = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
        mu.data(), mu.size());
    // Complementarity condition for gamma: mu lambda^n
    F.block(0, n_contacts, n_contacts, n_contacts) = mu_vec.asDiagonal();

    // Complementarity condition for gamma: lambda^t
    F.block(0, 2 * n_contacts, n_contacts,
            2 * n_contacts * num_friction_directions) = -E_t;

    // Complementarity condition for lambda_n: dt J_n (lambda^n component of
    // v_{k+1})
    F.block(n_contacts, n_contacts, n_contacts, n_contacts) =
        dt * dt * J_n * MinvJ_n_T;
    // Complementarity condition for lambda_n: dt J_n (lambda^t component of
    // v_{k+1})
    F.block(n_contacts, 2 * n_contacts, n_contacts,
            2 * n_contacts * num_friction_directions) =
        dt * dt * J_n * MinvJ_t_T;
    // Complementarity condition for lambda_t: dt J_t (gamma component of
    // v_{k+1})
    F.block(2 * n_contacts, 0, 2 * n_contacts * num_friction_directions,
            n_contacts) = E_t.transpose();
    // Complementarity condition for lambda_t: dt J_t (lambda^n component of
    // v_{k+1})
    F.block(2 * n_contacts, n_contacts,
            2 * n_contacts * num_friction_directions, n_contacts) =
        dt * J_t * MinvJ_n_T;
    // Complementarity condition for lambda_t: dt J_t (lambda^t component of
    // v_{k+1})
    F.block(2 * n_contacts, 2 * n_contacts,
            2 * n_contacts * num_friction_directions,
            2 * n_contacts * num_friction_directions) = dt * J_t * MinvJ_t_T;

    H.block(n_contacts, 0, n_contacts, n_u) = dt * dt * J_n * AB_v_u;
    H.block(2 * n_contacts, 0, 2 * n_contacts * num_friction_directions, n_u) =
        dt * J_t * AB_v_u;

    c.segment(n_contacts, n_contacts) = phi + dt * dt * J_n * d_v
        - J_n * vNqdot * plant.GetPositions(context);
    c.segment(2 * n_contacts, 2 * n_contacts * num_friction_directions) =
        J_t * dt * d_v;

  } else if (contact_model == ContactModel::kAnitescu) {
    VectorXd mu_vec = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
        mu.data(), mu.size());
    VectorXd anitescu_mu_vec = VectorXd::Zero(n_lambda);
    for (int i = 0; i < mu_vec.rows(); i++) {

      if (sampling_c3_options.with_z) {
        anitescu_mu_vec.segment((2 * num_friction_directions) * i,
                                2 * num_friction_directions) =
            mu_vec(i) * VectorXd::Ones(2 * num_friction_directions);

      }else {
        if (i == 0) {
          int tem_num_friction_directions = 1;
          anitescu_mu_vec.segment((2 * tem_num_friction_directions) * i,
                        2 * tem_num_friction_directions) =
          mu_vec(i) * VectorXd::Ones(2 * tem_num_friction_directions);
        }else {
          anitescu_mu_vec.segment((2 * num_friction_directions) * i -2,
                          2 * num_friction_directions) =
          mu_vec(i) * VectorXd::Ones(2 * num_friction_directions);

        }
      }

    }
    MatrixXd anitescu_mu_matrix = anitescu_mu_vec.asDiagonal();
    // Constructing friction bases
    MatrixXd J_c = E_t.transpose() * J_n + anitescu_mu_matrix * J_t;



    MatrixXd MinvJ_c_T = M_ldlt.solve(J_c.transpose());

    D.block(0, 0, n_q, n_lambda) = dt * dt * qdotNv * MinvJ_c_T;
    D.block(n_q, 0, n_v, n_lambda) = dt * MinvJ_c_T;

    // q component of complementarity constraint
    E.block(0, 0, n_lambda, n_q) =
        dt * J_c * AB_v_q + E_t.transpose() * J_n * vNqdot / dt;
    E.block(0, n_q, n_lambda, n_v) = J_c + dt * J_c * AB_v_v;

    // lambda component of complementarity constraint
    F = dt * J_c * MinvJ_c_T;

    std::cerr << "row of J_c" << J_c.rows() << "col of J_c" << J_c.cols() << std::endl;
    std::cerr << "row of MinvJ_c" << MinvJ_c_T.rows() << "col of MinvJ_c" << MinvJ_c_T.cols() << std::endl;

    // u component of complementarity constraint
    H = dt * J_c * AB_v_u;
    // constant component of complementarity constraint
    c = E_t.transpose() * phi / dt + dt * J_c * d_v -
        E_t.transpose() * J_n * vNqdot * plant.GetPositions(context) / dt;

    // Anitescu model needs an explicit formulation for the tangential
    // components in order to appropriately activate the robust constraint
    // (TODO): yangwill do another pass to verify this formulation
    W_x.block(0, 0, n_lambda, n_q) = J_t * AB_v_q;
    W_x.block(0, n_q, n_lambda, n_v) = J_t + J_t * AB_v_v;
    W_l = J_t * (MinvJ_c_T);
    W_u = J_t * (AB_v_u);
    w = J_t * (d_v);
  }


  LCS system(A, B, D, d, E, F, H, c, N, dt);
  return system;
}


LCS LCSFactory::LinearizePlantToLCS(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    const MultibodyPlant<AutoDiffXd>& plant_ad,
    const Context<AutoDiffXd>& context_ad,
    const vector<SortedPair<GeometryId>>& contact_geoms,
    int num_friction_directions, const std::vector<double>& mu, double dt,
    int N, ContactModel contact_model) {


  drake::yaml::LoadYamlOptions yaml_options;
  yaml_options.allow_yaml_with_no_cpp = true;

  std::string base_path = "/home/dair/opt/test/new_delet_z_sampling_c3/dairlib/examples/sampling_c3/push_t/";
  SamplingC3Options sampling_c3_options =
    drake::yaml::LoadYamlFile<SamplingC3Options>(
        base_path + "parameters/sampling_c3_options.yaml");



  int n_x = plant_ad.num_positions() + plant_ad.num_velocities();
  int n_u = plant_ad.num_actuators();


  int n_contacts = contact_geoms.size();

  DRAKE_DEMAND(plant_ad.num_velocities() == plant.num_velocities());
  DRAKE_DEMAND(plant_ad.num_positions() == plant.num_positions());
  DRAKE_DEMAND(mu.size() == n_contacts);
  int n_v = plant.num_velocities();
  int n_q = plant.num_positions();

  AutoDiffVecXd C(n_v);
  plant_ad.CalcBiasTerm(context_ad, &C);
  VectorXd u_dyn = plant.get_actuation_input_port().Eval(context);

  auto B_dyn_ad = plant_ad.MakeActuationMatrix();
  AutoDiffVecXd Bu =
      B_dyn_ad * plant_ad.get_actuation_input_port().Eval(context_ad);

  AutoDiffVecXd tau_g = plant_ad.CalcGravityGeneralizedForces(context_ad);

  drake::multibody::MultibodyForces<AutoDiffXd> f_app(plant_ad);
  plant_ad.CalcForceElementsContribution(context_ad, &f_app);

  MatrixX<AutoDiffXd> M(n_v, n_v);
  plant_ad.CalcMassMatrix(context_ad, &M);

  // If this ldlt is slow, there are alternate formulations which avoid it
  AutoDiffVecXd vdot_no_contact =
      M.ldlt().solve(tau_g + Bu + f_app.generalized_forces() - C);
  // Constant term in dynamics, d_vv = d + A x_0 + B u_0
  VectorXd d_vv = ExtractValue(vdot_no_contact);
  // Derivatives w.r.t. x and u, AB
  MatrixXd AB_v = ExtractGradient(vdot_no_contact);
  VectorXd x_dvv(n_q + n_v + n_u);
  x_dvv << plant.GetPositions(context), plant.GetVelocities(context), u_dyn;
  VectorXd x_dvvcomp = AB_v * x_dvv;
  VectorXd d_v = d_vv - x_dvvcomp;

  ///////////
  AutoDiffVecXd x_ad = plant_ad.GetPositionsAndVelocities(context_ad);
  AutoDiffVecXd qdot_no_contact(n_q);
  AutoDiffVecXd vel_ad = x_ad.tail(n_v);

  plant_ad.MapVelocityToQDot(context_ad, vel_ad, &qdot_no_contact);
  MatrixXd AB_q = ExtractGradient(qdot_no_contact);
  MatrixXd d_q = ExtractValue(qdot_no_contact);
  Eigen::SparseMatrix<double> Nqt;
  Nqt = plant.MakeVelocityToQDotMap(context);
  MatrixXd qdotNv = MatrixXd(Nqt);

  Eigen::SparseMatrix<double> NqI;
  NqI = plant.MakeQDotToVelocityMap(context);
  MatrixXd vNqdot = MatrixXd(NqI);

  VectorXd phi(n_contacts);
  MatrixXd J_n(n_contacts, n_v);
  MatrixXd J_t(2 * n_contacts * num_friction_directions, n_v);

  if (!sampling_c3_options.with_z) {
    if (n_contacts == 4) {
      J_t.resize(2 * (n_contacts-1) * num_friction_directions + num_friction_directions, n_v);
    }else if (n_contacts == 5){
      J_t.resize(2 * (n_contacts-2) * num_friction_directions + 2*num_friction_directions, n_v);
    }


  }

  std::cerr << "n_contact" << n_contacts << std::endl;
  for (int i = 0; i < n_contacts; i++) {
    multibody::GeomGeomCollider collider(
        plant,
        contact_geoms[i]);
    if (num_friction_directions == 1 || (!(sampling_c3_options.with_z) && ((i ==0 && n_contacts == 4) || (i <2  && n_contacts == 5)))) {
      Eigen::Vector3d planar_normal;
      int num_direction = 1;

      planar_normal << 0, 1, 0;
      auto [phi_i, J_i] = collider.EvalPlanar(context, planar_normal);
      phi(i) = phi_i;
      J_n.row(i) = J_i.row(0);
      J_t.block(2 * i * num_direction, 0, 2 * num_direction,
                n_v) = J_i.block(1, 0, 2 * num_direction, n_v);
    } else {
      auto [phi_i, J_i] =
          collider.EvalPolytope(context, num_friction_directions);
      phi(i) = phi_i;
      J_n.row(i) = J_i.row(0);
      if (sampling_c3_options.with_z){
        J_t.block(2 * i * num_friction_directions, 0, 2 * num_friction_directions,
                  n_v) = J_i.block(1, 0, 2 * num_friction_directions, n_v);
      }else {
        J_t.block(2 * i * num_friction_directions-2, 0, 2 * num_friction_directions,
          n_v) = J_i.block(1, 0, 2 * num_friction_directions, n_v);


      }
    }

    // J_i is 3 x n_v
    // row (0) is contact normal
    // rows (1-num_friction directions) are the contact tangents
  }

  auto M_ldlt = ExtractValue(M).ldlt();
  MatrixXd MinvJ_n_T = M_ldlt.solve(J_n.transpose());
  MatrixXd MinvJ_t_T = M_ldlt.solve(J_t.transpose());

  MatrixXd A = MatrixXd::Zero(n_x, n_x);
  MatrixXd B = MatrixXd::Zero(n_x, n_u);
  VectorXd d = VectorXd::Zero(n_x);

  MatrixXd AB_v_q = AB_v.block(0, 0, n_v, n_q);
  MatrixXd AB_v_v = AB_v.block(0, n_q, n_v, n_v);
  MatrixXd AB_v_u = AB_v.block(0, n_x, n_v, n_u);
  MatrixXd M_double = MatrixXd::Zero(n_v, n_v);
  plant.CalcMassMatrix(context, &M_double);

  A.block(0, 0, n_q, n_q) =
      MatrixXd::Identity(n_q, n_q) + dt * dt * qdotNv * AB_v_q;
  A.block(0, n_q, n_q, n_v) = dt * qdotNv + dt * dt * qdotNv * AB_v_v;
  A.block(n_q, 0, n_v, n_q) = dt * AB_v_q;
  A.block(n_q, n_q, n_v, n_v) = dt * AB_v_v + MatrixXd::Identity(n_v, n_v);

  B.block(0, 0, n_q, n_u) = dt * dt * qdotNv * AB_v_u;
  B.block(n_q, 0, n_v, n_u) = dt * AB_v_u;

  d.head(n_q) = dt * dt * qdotNv * d_v;
  d.tail(n_v) = dt * d_v;

  MatrixXd E_t =
      MatrixXd::Zero(n_contacts, 2 * n_contacts * num_friction_directions);


  if (!sampling_c3_options.with_z) {
    E_t.resize(n_contacts, 2 * n_contacts * num_friction_directions - 2);
  }

  for (int i = 0; i < n_contacts; i++) {

    if (sampling_c3_options.with_z) {
      E_t.block(i, i * (2 * num_friction_directions), 1,
                2 * num_friction_directions) =
          MatrixXd::Ones(1, 2 * num_friction_directions);

    }else {
      if ((i == 0&& n_contacts ==4)|| (i<2 && n_contacts ==5)){
        int temp_num_friction_directions = 1;

        E_t.block(i, i * (2 * temp_num_friction_directions), 1,
          2 * temp_num_friction_directions) =
        MatrixXd::Ones(1, 2 * temp_num_friction_directions);
      }else {
        E_t.block(i, i * (2 * num_friction_directions)-2, 1,
          2 * num_friction_directions) =
        MatrixXd::Ones(1, 2 * num_friction_directions);

      }

    }


  }

  int n_lambda = 0;
  if (contact_model == ContactModel::kStewartAndTrinkle) {
    n_lambda = 2 * n_contacts + 2 * n_contacts * num_friction_directions;
  } else {
    if (sampling_c3_options.with_z) {
      n_lambda = 2 * n_contacts * num_friction_directions;
    }else {
      n_lambda = 2 * n_contacts * num_friction_directions -2;
    }

  }


  //std::cerr << "lambda in lcs factory" << n_lambda << std::endl;

  //std::cerr << "n_contacts" << n_contacts << std::endl;

  //std::cerr << "num_friction_directions" << num_friction_directions << std::endl;



  // Matrices with contact variables
  MatrixXd D = MatrixXd::Zero(n_x, n_lambda);
  MatrixXd E = MatrixXd::Zero(n_lambda, n_x);
  MatrixXd F = MatrixXd::Zero(n_lambda, n_lambda);
  MatrixXd H = MatrixXd::Zero(n_lambda, n_u);
  VectorXd c = VectorXd::Zero(n_lambda);

  MatrixXd W_x = MatrixXd::Zero(n_lambda, n_x);
  MatrixXd W_l = MatrixXd::Zero(n_lambda, n_lambda);
  MatrixXd W_u = MatrixXd::Zero(n_lambda, n_u);
  MatrixXd w = VectorXd::Zero(n_lambda);

  if (contact_model == ContactModel::kStewartAndTrinkle) {
    D.block(0, 2 * n_contacts, n_q, 2 * n_contacts * num_friction_directions) =
        dt * dt * qdotNv * MinvJ_t_T;
    D.block(n_q, 2 * n_contacts, n_v,
            2 * n_contacts * num_friction_directions) = dt * MinvJ_t_T;
    D.block(0, n_contacts, n_q, n_contacts) = dt * dt * qdotNv * MinvJ_n_T;

    D.block(n_q, n_contacts, n_v, n_contacts) = dt * MinvJ_n_T;
    // Complementarity condition for gamma: mu lambda^n
    E.block(n_contacts, 0, n_contacts, n_q) =
        dt * dt * J_n * AB_v_q + J_n * vNqdot;
    E.block(2 * n_contacts, 0, 2 * n_contacts * num_friction_directions, n_q) =
        dt * J_t * AB_v_q;
    E.block(n_contacts, n_q, n_contacts, n_v) =
        dt * J_n + dt * dt * J_n * AB_v_v;
    E.block(2 * n_contacts, n_q, 2 * n_contacts * num_friction_directions,
            n_v) = J_t + dt * J_t * AB_v_v;

    VectorXd mu_vec = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
        mu.data(), mu.size());
    // Complementarity condition for gamma: mu lambda^n
    F.block(0, n_contacts, n_contacts, n_contacts) = mu_vec.asDiagonal();

    // Complementarity condition for gamma: lambda^t
    F.block(0, 2 * n_contacts, n_contacts,
            2 * n_contacts * num_friction_directions) = -E_t;

    // Complementarity condition for lambda_n: dt J_n (lambda^n component of
    // v_{k+1})
    F.block(n_contacts, n_contacts, n_contacts, n_contacts) =
        dt * dt * J_n * MinvJ_n_T;
    // Complementarity condition for lambda_n: dt J_n (lambda^t component of
    // v_{k+1})
    F.block(n_contacts, 2 * n_contacts, n_contacts,
            2 * n_contacts * num_friction_directions) =
        dt * dt * J_n * MinvJ_t_T;
    // Complementarity condition for lambda_t: dt J_t (gamma component of
    // v_{k+1})
    F.block(2 * n_contacts, 0, 2 * n_contacts * num_friction_directions,
            n_contacts) = E_t.transpose();
    // Complementarity condition for lambda_t: dt J_t (lambda^n component of
    // v_{k+1})
    F.block(2 * n_contacts, n_contacts,
            2 * n_contacts * num_friction_directions, n_contacts) =
        dt * J_t * MinvJ_n_T;
    // Complementarity condition for lambda_t: dt J_t (lambda^t component of
    // v_{k+1})
    F.block(2 * n_contacts, 2 * n_contacts,
            2 * n_contacts * num_friction_directions,
            2 * n_contacts * num_friction_directions) = dt * J_t * MinvJ_t_T;

    H.block(n_contacts, 0, n_contacts, n_u) = dt * dt * J_n * AB_v_u;
    H.block(2 * n_contacts, 0, 2 * n_contacts * num_friction_directions, n_u) =
        dt * J_t * AB_v_u;

    c.segment(n_contacts, n_contacts) = phi + dt * dt * J_n * d_v
        - J_n * vNqdot * plant.GetPositions(context);
    c.segment(2 * n_contacts, 2 * n_contacts * num_friction_directions) =
        J_t * dt * d_v;

  } else if (contact_model == ContactModel::kAnitescu) {
    VectorXd mu_vec = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
        mu.data(), mu.size());
    VectorXd anitescu_mu_vec = VectorXd::Zero(n_lambda);
    for (int i = 0; i < mu_vec.rows(); i++) {

      if (sampling_c3_options.with_z) {
        anitescu_mu_vec.segment((2 * num_friction_directions) * i,
                                2 * num_friction_directions) =
            mu_vec(i) * VectorXd::Ones(2 * num_friction_directions);

      }else {
        if (i == 0) {
          int tem_num_friction_directions = 1;
          anitescu_mu_vec.segment((2 * tem_num_friction_directions) * i,
                        2 * tem_num_friction_directions) =
          mu_vec(i) * VectorXd::Ones(2 * tem_num_friction_directions);
        }else {
          anitescu_mu_vec.segment((2 * num_friction_directions) * i -2,
                          2 * num_friction_directions) =
          mu_vec(i) * VectorXd::Ones(2 * num_friction_directions);

        }
      }

    }
    MatrixXd anitescu_mu_matrix = anitescu_mu_vec.asDiagonal();
    // Constructing friction bases
    MatrixXd J_c = E_t.transpose() * J_n + anitescu_mu_matrix * J_t;



    MatrixXd MinvJ_c_T = M_ldlt.solve(J_c.transpose());

    D.block(0, 0, n_q, n_lambda) = dt * dt * qdotNv * MinvJ_c_T;
    D.block(n_q, 0, n_v, n_lambda) = dt * MinvJ_c_T;

    // q component of complementarity constraint
    E.block(0, 0, n_lambda, n_q) =
        dt * J_c * AB_v_q + E_t.transpose() * J_n * vNqdot / dt;
    E.block(0, n_q, n_lambda, n_v) = J_c + dt * J_c * AB_v_v;

    // lambda component of complementarity constraint
    F = dt * J_c * MinvJ_c_T;

    //std::cerr << "row of J_c" << J_c.rows() << "col of J_c" << J_c.cols() << std::endl;
    //std::cerr << "row of MinvJ_c" << MinvJ_c_T.rows() << "col of MinvJ_c" << MinvJ_c_T.cols() << std::endl;

    // u component of complementarity constraint
    H = dt * J_c * AB_v_u;
    // constant component of complementarity constraint
    c = E_t.transpose() * phi / dt + dt * J_c * d_v -
        E_t.transpose() * J_n * vNqdot * plant.GetPositions(context) / dt;

    // Anitescu model needs an explicit formulation for the tangential
    // components in order to appropriately activate the robust constraint
    // (TODO): yangwill do another pass to verify this formulation
    W_x.block(0, 0, n_lambda, n_q) = J_t * AB_v_q;
    W_x.block(0, n_q, n_lambda, n_v) = J_t + J_t * AB_v_v;
    W_l = J_t * (MinvJ_c_T);
    W_u = J_t * (AB_v_u);
    w = J_t * (d_v);
  }


  LCS system(A, B, D, d, E, F, H, c, N, dt);
  return system;
}

