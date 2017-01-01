within ;
package Version_7

  package Experiment

    model Experiment_1
      Version_7.Vehicles.vehicle vehicle;
      constant Real pi = Modelica.Constants.pi;
      Real swdamp = (110 * pi) / 180 "amplitude of the swd manouvre";
    equation
      vehicle.Aped =if time<5 then 0 elseif time< 7 then 40 elseif time < 15 then 20 else  0;
      vehicle.Bped = if time < 15 then 0 else if time < 20 then 100 else 0;
      vehicle.trans.r_gear = if time<5 then 0 elseif time< 20 then 0 else 0;
      vehicle.chassis.theta = 0;
      vehicle.chassis.swa = 0;//  if time < 10 then 0 elseif time < 10 + (1 / 0.7 * 3) / 4 then swdamp * sin(2 * pi * 0.7 * (time - 10)) elseif time < 10 + (1 / 0.7 * 3) / 4 + 0.5 then -swdamp elseif time < 10 + (1 / 0.7 * 4) / 4 + 0.5 then swdamp * sin(2 * pi * 0.7 * (time - 10 - 0.5)) else 0;
      //vehicle.Tacclim = 1500;
      //if time < 10 then 0 elseif time < 10 + (1 / 0.7 * 3) / 4 then swdamp * sin(2 * pi * 0.7 * (time - 10)) elseif time < 10 + (1 / 0.7 * 3) / 4 + 0.5 then -swdamp elseif time < 10 + (1 / 0.7 * 4) / 4 + 0.5 then swdamp * sin(2 * pi * 0.7 * (time - 10 - 0.5)) else 0;
      annotation (experiment(StartTime=0, StopTime=50,fixedstepsize=0.0005,
          Algorithm="Dassl"),Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Experiment_1;
  end Experiment;

  package Vehicles

    model vehicle_1
      Version_7.Driveline.transmission_modular_1 trans(m=chassis.m, st_agear=
            st_agear);
      Version_7.body_and_wheel.chassis chassis;
      Version_7.body_and_wheel.wheel wheel1;
      Version_7.body_and_wheel.wheel wheel2;
      input Real Aped;
      input Real Bped;
      input Real swa;
      input Real theta;
      input Real r_gear;
      parameter Integer st_agear= 3;
      output Real vx;
      output Real ax;
      output Integer agear;

    equation
      //Forces
      wheel1.Fxv = chassis.Fxf;
      wheel2.Fxv = chassis.Fxr;
      wheel1.Fyv = chassis.Fyf;
      wheel2.Fyv = chassis.Fyr;
      wheel1.Fz = chassis.Fzf;
      wheel2.Fz = chassis.Fzr;

      //Velocity
      wheel1.Vx = chassis.vx;
      wheel2.Vx = chassis.vx;
      wheel1.Vy = chassis.vy + chassis.yaw_velocity*chassis.a;
      wheel2.Vy = chassis.vy - chassis.yaw_velocity*chassis.b;

      wheel1.delta = chassis.delta;
      wheel2.delta = 0;

      wheel1.Brake_torque = chassis.Tbw;
      wheel2.Brake_torque = chassis.Tbw;

      chassis.womega = (wheel1.omega + wheel2.omega)/2;

      // powertrain
      r_gear = trans.r_gear;
      wheel1.Drive_torque = trans.M_wheel_1;
      wheel2.Drive_torque = if r_gear == 1 then trans.rev_torque else trans.M_wheel_2;
      chassis.vx = trans.Vx;
      wheel1.omega = trans.w_wheel_1;
      wheel2.omega = trans.w_wheel_2;
      wheel1.i_T = 0;
      wheel2.i_T = trans.i_T;
      wheel1.Je = 0;
      wheel2.Je = chassis.Je;
      trans.Je = chassis.Je;
      trans.Fx = chassis.Fx;

      //
      Bped = chassis.brakeped;
      trans.Aped = Aped;
      swa= chassis.swa;
      theta = chassis.theta;

      vx = chassis.vx;
      ax = chassis.ax;
      agear = trans.agear;
       annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end vehicle_1;

    model vehicle_2
      Version_7.Driveline.transmission_modular_2 trans(m=chassis.m, st_agear=
            st_agear);
      Version_7.body_and_wheel.chassis chassis;
      Version_7.body_and_wheel.wheel wheel1;
      Version_7.body_and_wheel.wheel wheel2;
      input Real Aped;
      input Real Bped;
      input Real swa;
      input Real theta;
      input Real r_gear;
      parameter Integer st_agear= 3;
      output Real vx;
      output Real Fxr;
      output Real x;
      output Real y;
      output Real ax;
      output Real Fyf;
      output Real delta;
      output Real decl;
      output Integer agear;
      output Real ay;
      output Real yaw_velocity;
    equation
      //Forces
      wheel1.Fxv = chassis.Fxf;
      wheel2.Fxv = chassis.Fxr;
      wheel1.Fyv = chassis.Fyf;
      wheel2.Fyv = chassis.Fyr;
      wheel1.Fz = chassis.Fzf;
      wheel2.Fz = chassis.Fzr;

      //Velocity
      wheel1.Vx = chassis.vx;
      wheel2.Vx = chassis.vx;
      wheel1.Vy = chassis.vy + chassis.yaw_velocity*chassis.a;
      wheel2.Vy = chassis.vy - chassis.yaw_velocity*chassis.b;

      wheel1.delta = chassis.delta;
      wheel2.delta = 0;

      wheel1.Brake_torque = chassis.Tbw;
      wheel2.Brake_torque = chassis.Tbw;

      chassis.womega = (wheel1.omega + wheel2.omega)/2;

      // powertrain
      r_gear = trans.r_gear;
      wheel1.Drive_torque = trans.M_wheel_1;
      wheel2.Drive_torque = if r_gear == 1 then trans.rev_torque else trans.M_wheel_2;
      chassis.vx = trans.Vx;
      wheel1.omega = trans.w_wheel_1;
      wheel2.omega = trans.w_wheel_2;
      wheel1.i_T = 0;
      wheel2.i_T = trans.i_T;
      wheel1.Je = 0;
      wheel2.Je = chassis.Je;
      trans.Je = chassis.Je;
      trans.Fx = chassis.Fx;

      //
      Bped = chassis.brakeped;
      trans.Aped = Aped;
      swa= chassis.swa;
      theta = chassis.theta;
      Fxr = wheel2.Fxv;
      Fyf = wheel1.Fyv;
      delta = chassis.delta;
      decl = chassis.decl;
      x = chassis.x;
      y = chassis.y;
      vx = chassis.vx;
      ax = chassis.ax;
      agear = trans.agear;
      ay = chassis.ay;
      yaw_velocity = chassis.yaw_velocity;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end vehicle_2;

    model vehicle
      Version_7.Driveline.transmission_modular trans(m=chassis.m, st_agear=
            st_agear);
      Version_7.body_and_wheel.chassis chassis;
      Version_7.body_and_wheel.wheel wheel1;
      Version_7.body_and_wheel.wheel wheel2;
      input Real Aped;
      input Real Bped;
      input Real swa;
      input Real theta;
      input Real r_gear;
      parameter Integer st_agear= 4;
      output Real vx;
      output Real Fxr;
      output Real x;
      output Real y;
      output Real ax;
      output Real Fyf;
      output Real delta;
      output Real decl;
      output Integer agear;
      output Real ay;
      output Real yaw_velocity;
    equation
      //Forces
      wheel1.Fxv = chassis.Fxf;
      wheel2.Fxv = chassis.Fxr;
      wheel1.Fyv = chassis.Fyf;
      wheel2.Fyv = chassis.Fyr;
      wheel1.Fz = chassis.Fzf;
      wheel2.Fz = chassis.Fzr;

      //Velocity
      wheel1.Vx = chassis.vx;
      wheel2.Vx = chassis.vx;
      wheel1.Vy = chassis.vy + chassis.yaw_velocity*chassis.a;
      wheel2.Vy = chassis.vy - chassis.yaw_velocity*chassis.b;

      wheel1.delta = chassis.delta;
      wheel2.delta = 0;

      wheel1.Brake_torque = chassis.Tbw;
      wheel2.Brake_torque = chassis.Tbw;

      chassis.womega = (wheel1.omega + wheel2.omega)/2;

      // powertrain
      r_gear = trans.r_gear;
      wheel1.Drive_torque = trans.M_wheel_1;
      wheel2.Drive_torque = if r_gear == 1 then trans.rev_torque else trans.M_wheel_2;
      chassis.vx = trans.Vx;
      wheel1.omega = trans.w_wheel_1;
      wheel2.omega = trans.w_wheel_2;
      wheel1.i_T = 0;
      wheel2.i_T = trans.i_T;
      wheel1.Je = 0;
      wheel2.Je = chassis.Je;
      trans.Je = chassis.Je;
      trans.Fx = chassis.Fx;

      //
      Bped = chassis.brakeped;
      trans.Aped = Aped;
      swa= chassis.swa;
      theta = chassis.theta;
      Fxr = wheel2.Fxv;
      Fyf = wheel1.Fyv;
      delta = chassis.delta;
      decl = chassis.decl;
      x = chassis.x;
      y = chassis.y;
      vx = chassis.vx;
      ax = chassis.ax;
      agear = trans.agear;
      ay = chassis.ay;
      yaw_velocity = chassis.yaw_velocity;
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end vehicle;

    model vehicle_3
      Version_7.Driveline.transmission_modular_3 trans(m=chassis.m, st_agear=
            st_agear);
      Version_7.body_and_wheel.chassis chassis;
      Version_7.body_and_wheel.wheel wheel1;
      Version_7.body_and_wheel.wheel wheel2;
      Real v;
      input Real Aped;
      input Real Bped;
      input Real swa;
      input Real theta;
      input Real r_gear;
      input Real Tacclim;
      input Real check_1;
      parameter Integer st_agear= 4;
      output Real vx;
      output Real Fxr;
      output Real x;
      output Real y;
      output Real ax;
      output Real Fyf;
      output Real delta;
      output Real decl;
      output Integer agear;
      output Real ay;
      output Real yaw_velocity;

    equation
      //Forces
      wheel1.Fxv = chassis.Fxf;
      wheel2.Fxv = chassis.Fxr;
      wheel1.Fyv = chassis.Fyf;
      wheel2.Fyv = chassis.Fyr;
      wheel1.Fz = chassis.Fzf;
      wheel2.Fz = chassis.Fzr;

      //Velocity
      wheel1.Vx = chassis.vx;
      wheel2.Vx = chassis.vx;
      wheel1.Vy = chassis.vy + chassis.yaw_velocity*chassis.a;
      wheel2.Vy = chassis.vy - chassis.yaw_velocity*chassis.b;

      wheel1.delta = chassis.delta;
      wheel2.delta = 0;

      wheel1.Brake_torque = chassis.Tbw;
      wheel2.Brake_torque = chassis.Tbw;

      chassis.womega = (wheel1.omega + wheel2.omega)/2;

      // powertrain
      r_gear = trans.r_gear;
      wheel1.Drive_torque = trans.M_wheel_1;
      wheel2.Drive_torque = if r_gear == 1 then trans.rev_torque else trans.M_wheel_2;
      chassis.vx = trans.Vx;
      wheel1.omega = trans.w_wheel_1;
      wheel2.omega = trans.w_wheel_2;
      wheel1.i_T = 0;
      wheel2.i_T = trans.i_T;
      wheel1.Je = 0;
      wheel2.Je = chassis.Je;
      trans.Je = chassis.Je;
      trans.Fx = chassis.Fx;
      trans.Ta = Tacclim;
      //
      Bped = chassis.brakeped;
      trans.Aped = Aped;
      swa= chassis.swa;
      theta = chassis.theta;
      Fxr = wheel2.Fxv;
      Fyf = wheel1.Fyv;
      delta = chassis.delta;
      decl = chassis.decl;
      x = chassis.x;
      y = chassis.y;
      vx = chassis.vx;
      ax = chassis.ax;
      agear = trans.agear;
      ay = chassis.ay;
      yaw_velocity = chassis.yaw_velocity;
      v = min(check_1,1);

         annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end vehicle_3;
  end Vehicles;

  package Driveline

    model engine_modular
      import SI = Modelica.SIunits;
      constant Real pi = Modelica.Constants.pi;
      parameter  SI.Mass m;

      //Engine coefficients
      parameter Real k3max = -1.246e-06;
      parameter Real k2max = 1.471e-04;
      parameter Real k1max = 6.5110;
      parameter Real k0max = -2.2287e+03;
      parameter Real k3min = -1.59e-8;
      parameter Real k2min = 8.7520e-5;
      parameter Real k1min = -0.197569;
      parameter Real k0min = -4.22681;
      parameter SI.AngularVelocity w_eng_max = 2000*pi/30
        "maximum engine rotation";
      parameter SI.AngularVelocity w_eng_idl = 600*pi/30 " engine idle speed";
      parameter SI.Acceleration axh = 0.7 "Upper acceleration limit";
      parameter SI.Acceleration axl = 0.6 "Lower acceleration limit";
      parameter Real E_factor = 0.45;
      parameter SI.Torque Tsplit = 700;
      parameter Real k = 1 "Boost pressure coefficient";
      Real perc_throttle;
      Real Aped "Accelerator pedal position [%]";
      SI.Velocity Vx "velocity of the vehicle m/s";
      SI.Torque torque(start = 0) "engine torque";
      SI.Torque Te;
      SI.Torque max_torque;
      SI.Torque max_torque_alim;
      SI.Torque min_torque;
      SI.Torque Tbase;
      SI.Torque Tdynreq;
      SI.Torque Ttop;
      SI.Torque T_e;
      SI.Force Fx "Force to compute ax";
      SI.AngularVelocity w_engine(start = w_eng_idl) "engine speed in rad/s";
      SI.AngularVelocity rpm_engine "engine speed in rpm";
       Modelica.Blocks.Tables.CombiTable1Ds tab_torque(table = [0,0;
                                                                 62.8318,1660;
                                                                 73.3038,1880;
                                                                 83.775,2240;
                                                                 94.247,2900;
                                                                 104.719,3550;
                                                                 115.191,3550;
                                                                 125.663,3550;
                                                                 136.135,3550;
                                                                 146.607,3550;
                                                                 157.079,3470;
                                                                 167.551,3310;
                                                                 178.023,3120;
                                                                 188.495,2880;
                                                                 198.967,2660;
                                                                 209.439,1680;
                                                                 219.911,0],smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments);
    equation
      rpm_engine = w_engine * 30 / pi;
      tab_torque.u = w_engine;
      tab_torque.y[1] = max_torque;
      max_torque_alim = if der(Vx) > axh then E_factor*max_torque else if der(Vx) < axl then max_torque else max_torque*( (((der(Vx)-axl)*(E_factor-1))/(axh-axl)) + 1);
      min_torque = k3min * rpm_engine ^ 3 + k2min * rpm_engine ^ 2 + k1min * rpm_engine + k0min;
      perc_throttle = max(min(Aped/100, 1), 0.0);
      torque = (perc_throttle * ((max_torque_alim) - min_torque) + min_torque) * (1 - tanh((w_engine - w_eng_max) * 0.01));
      Tbase = min(torque,Tsplit);
      Tdynreq = torque - Tbase;
      der(Ttop) = k*(Tdynreq - Ttop);
      T_e = Tbase + Ttop;
      Te = noEvent(if Vx < 0 then max(0, T_e) else T_e);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end engine_modular;

    model transmission_modular
      extends Version_7.Driveline.engine_modular;
      import SI = Modelica.SIunits;
      constant Real pi = Modelica.Constants.pi;
       // transmission parameters
      parameter Real i_tm[12] = {14.356,11.7239, 9.0363, 7.0941, 5.5382, 4.3478, 3.4363,  2.6977, 2.0783, 1.6316, 1.2738, 1}
        "gear ratios from 1st up to 12th gear";
      parameter Real i_final = 3.46 "gear ratio final gear";
      parameter Real eff_tr = 0.90 "transmission efficiency";
      parameter Real upshift[12] = {2.31,3,3.81,4.86,6.25,8,10.02,12.8,16.65,21.2,27.1,1000000};
      parameter Real downshift[12] = {-1000000,2.2,2.28,3.5,4.5,6,7.5,9.4,12,15.5,20,25};
      parameter SI.Torque max_drive_torque = 12e5;
      parameter Real dds=5 "damping factor for drive shaft flexibility ";
      parameter Integer st_agear;
      Integer agear(start = st_agear) " the automatic gear";
      Real i_T(start = 0) "total transmission ratio";
      Real r_gear;
      Real Je;
      SI.Torque Tc;
      SI.Torque Tt;
      SI.Torque Tp;
      SI.Torque Tf;
      SI.Torque Td;
      SI.Torque Tw;
      SI.AngularVelocity w_c;
      SI.AngularVelocity w_t;
      SI.AngularVelocity w_p;
      SI.AngularVelocity w_f;
      SI.AngularVelocity w_w;

      SI.Torque M_wheel_1;
      SI.Torque M_wheel_2;
      SI.AngularVelocity w_wheel_1;
      SI.AngularVelocity w_wheel_2;
      SI.AngularVelocity w_mean_wheels;
      SI.Torque rev_torque;

      output Real r_c;
    equation

      i_T = i_final * i_tm[agear];
      when Vx > upshift[pre(agear)] then
        agear = pre(agear)+1;
      elsewhen Vx < downshift[pre(agear)] then
        agear = pre(agear)-1;
      end when;

      Je*der(w_engine) = Te- Tc; // Torque balance according to Newtons law
      w_engine = min(w_eng_max,max(w_c,w_eng_idl));

      Tc = Tt;

      w_c = w_t;// the speed of the clutch would be reduced by the gear ratio whcih gives the transmission speed
      Tp = Tt*i_tm[agear];// The torque leaving the

      w_t/i_tm[agear] = w_p; // the speed coming out of the tranmission will be the speed of the propeller shaft

      Tp = Tf;// torque that is entering the final drive is the torque on the propeller shaft

      w_p = w_f;// the speed after the final drive is w_f which is the propeller shaft speed w_p reduced by the final drive ratio
      eff_tr*Tf*i_final = Td;// the torque that comes into the final drive is Tf and is multiplied by i_final to give the Td which is leaving the final drive sent out on to the drive shaft

      //Td = Tw + dds*(w_f - w_w);
    //   der(Td) = Td- Tw+dds*(w_f-w_w);

      Td = Tw; // the drive shaft will be torque that will be exerted on the torque without driveshaft flexibiltiy

      w_f/i_final = w_w; // the speed from the final drive is wheel speed
      w_w = w_mean_wheels;
      w_mean_wheels = (w_wheel_1+w_wheel_2)/2;
      rev_torque = -1*r_c*max(Tt,0)*i_tm[6]*i_final;
      r_c = if r_gear > 0 then 1 else 0;
      M_wheel_2 = max(Tw,0);//fix for - ve velocity on braking
      M_wheel_1 = 0;

        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end transmission_modular;

    model engine_modular_1
      import SI = Modelica.SIunits;
      constant Real pi = Modelica.Constants.pi;
      Real Aped "Aceelerator pedal[%]";
      SI.AngularVelocity w_engine "Engine speed";
      SI.AngularVelocity w_engine_en "Engine speed";
      SI.Torque T_raw "Raw engine speed";
      SI.Torque T_req_max "Required Max torque";
      SI.Torque T_req "Required torque";
      SI.Torque T_ss "Steady State torque";
      SI.Torque Tdynreq;
      SI.Torque Ttop;
      SI.Torque Te;
      SI.Torque T_e;
      SI.Torque Tbase;
      SI.Force Fx "Force to compute ax";
      Real Aped_en "Aped to the engine in b/w 0 to 1";
      Real q "Fuel parmeter";
      parameter SI.Acceleration axh = 0.7 "Upper acceleration limit";
      parameter SI.Acceleration axl = 0.6 "Lower acceleration limit";
      parameter Real E_factor = 0.15;
      parameter SI.Torque Tsplit = 700;
      parameter  SI.Mass m=9814;
      parameter Real k = 1 "Boost pressure coefficient";
      Modelica.Blocks.Tables.CombiTable2D engine_map(table = [0,20.943,62.831,73.303,78.539,83.775,89.011,94.247,99.483,104.719,109.95,115.191,125.663,136.135,146.607,151.84,157.079,167.551,178.023,188.495,198.967,209.439,219.911,230.383;
                                                              0,0,-300,-300,-300,-300,-300,-300,-300,-300,-300,-300,-300,-300,-300,-300,-300,-300,-300,-300,-300,-300,-300,-300;
                                                              0.1,0,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10;
                                                              0.2,0,280,280,280,280,280,280,280,280,280,280,280,280,280,280,280,280,280,280,280,280,280,280;
                                                              0.3,0,570,570,570,570,570,570,570,570,570,570,570,570,570,570,570,570,570,570,570,570,570,570;
                                                              0.4,0,860,860,860,860,860,860,860,860,860,860,860,860,860,860,860,860,860,860,860,860,860,860;
                                                              0.5,0,1600,1600,1600,1600,1600,1600,1600,1600,1600,1600,1600,1600,1600,1600,1600,1600,1600,1600,1600,1600,1600,1600;
                                                              0.6,0,2400,2400,2400,2400,2400,2400,2400,2400,2400,2400,2400,2400,2400,2400,2400,2400,2400,2400,2400,2400,2400,2400;
                                                              0.7,0,2740,2740,2740,2740,2740,2740,2740,2740,2740,2740,2740,2740,2740,2740,2740,2740,2740,2740,2740,2740,2740,2740;
                                                              0.8,0,2960,2960,2960,2960,2960,2960,2960,2960,2960,2960,2960,2960,2960,2960,2960,2960,2960,2960,2960,2960,2960,2960;
                                                              0.9,0,3200,3200,3200,3200,3200,3200,3200,3200,3200,3200,3200,3200,3200,3200,3200,3200,3200,3200,3200,3200,3200,3200;
                                                              1.0,0,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550], smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments) "u1:throttle, u2:rps, y:torque";
                                                              //The engine map is from par VehProp model please check if it is the 750HP models as Rhino
      Modelica.Blocks.Tables.CombiTable1Ds max_torque(table = [0,0;
                                                              62.8318,1660;
                                                              73.3038,1880;
                                                              83.775,2240;
                                                              94.247,2900;
                                                              104.719,3550;
                                                              115.191,3550;
                                                              125.663,3550;
                                                              136.135,3550;
                                                              146.607,3550;
                                                              157.079,3470;
                                                              167.551,3310;
                                                              178.023,3120;
                                                              188.495,2880;
                                                              198.967,2660;
                                                              209.439,1680;
                                                              219.911,0],smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments);
      Modelica.Blocks.Tables.CombiTable2D Fuel_map(table = [0,-104.71,78.539,94.247,109.95,125.663,141.137,157.079,172.78,188.495,204.439,219.911;
                                                            0,0,0,0,0,0,0,0,0,0,0,0;
                                                            260,25,25,25,25,25,25,25,25,25,25,25;
                                                            520,50,50,50,50,50,50,50,50,50,50,50;
                                                            780,75,75,75,75,75,75,75,75,75,75,75;
                                                            1040,100,100,100,100,100,100,100,100,100,100,100;
                                                            1640,125,125,125,125,125,125,125,125,125,125,125;
                                                            2000,150,150,150,150,150,150,150,150,150,150,150;
                                                            2440,175,175,175,175,175,175,175,175,175,175,175;
                                                            2800,200,200,200,200,200,200,200,200,200,200,200;
                                                            3130,225,225,225,225,225,225,225,225,225,225,225;
                                                            3550,250,250,250,250,250,250,250,250,250,250,250], smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments) "u1:Torque, u2:rps, y:q";
      Modelica.Blocks.Tables.CombiTable2D Steadystatetorque(table = [0,-104.71,78.539,94.247,109.95,125.663,141.137,157.079,172.78,188.495,204.439,219.911;
                                                            0,0,0,0,0,0,0,0,0,0,0,0;
                                                            25,260,260,260,260,260,260,260,260,260,260,260;
                                                            50,520,520,520,520,520,520,520,520,520,520,520;
                                                            75,780,780,780,780,780,780,780,780,780,780,780;
                                                            100,1040,1040,1040,1040,1040,1040,1040,1040,1040,1040,1040;
                                                            125,1640,1640,1640,1640,1640,1640,1640,1640,1640,1640,1640;
                                                            150,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000,2000;
                                                            175,2440,2440,2440,2440,2440,2440,2440,2440,2440,2440,2440;
                                                            200,2800,2800,2800,2800,2800,2800,2800,2800,2800,2800,2800;
                                                            225,3130,3130,3130,3130,3130,3130,3130,3130,3130,3130,3130;
                                                            250,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550,3550], smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments) "u1:q, u2:rps, y:torque";

      SI.Velocity Vx "velocity of the vehicle m/s";
      SI.Torque T_req_new;
      parameter SI.AngularVelocity w_eng_max = 2000*pi/30
        "maximum engine rotation";
      parameter SI.AngularVelocity w_eng_idl = 600*pi/30 " engine idle speed";

    equation
      Aped_en = max(min(1.0,Aped/100),0.0);
      w_engine_en = max(min(w_eng_max,w_engine),w_eng_idl);
      engine_map.u1 = Aped_en;
      engine_map.u2 = w_engine;
      T_raw = engine_map.y;
      max_torque.u = w_engine_en;
      max_torque.y[1] = T_req_max;
      T_req = min(T_raw,T_req_max);
      T_req_new = if der(Vx) > axh then E_factor*T_req else if der(Vx) < axl then T_req else T_req*( (((der(Vx)-axl)*(E_factor-1))/(axh-axl)) + 1);
      Fuel_map.u1 = T_req;
      Fuel_map.u2 = w_engine_en;
      Fuel_map.y = q;
      Steadystatetorque.u1 = q;
      Steadystatetorque.u2 = w_engine_en;
      Steadystatetorque.y = T_ss;
      Tbase = min(T_ss,Tsplit);
      Tdynreq = T_ss - Tbase;
      der(Ttop) = k*(Tdynreq - Ttop);
      T_e = Tbase + Ttop;
      Te = noEvent(if Vx < 0 then max(0, T_e) else T_e);
      //Te = T_e;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end engine_modular_1;

    model engine_modular_2
      import SI = Modelica.SIunits;
      constant Real pi = Modelica.Constants.pi;
      parameter  SI.Mass m;

      //Engine coefficients
      parameter Real k3max = -1.246e-06;
      parameter Real k2max = 1.471e-04;
      parameter Real k1max = 6.5110;
      parameter Real k0max = -2.2287e+03;
      parameter Real k3min = -1.59e-8;
      parameter Real k2min = 8.7520e-5;
      parameter Real k1min = -0.197569;
      parameter Real k0min = -4.22681;
      parameter SI.AngularVelocity w_eng_max = 2000*pi/30
        "maximum engine rotation";
      parameter SI.AngularVelocity w_eng_idl = 600*pi/30 " engine idle speed";
      parameter SI.Acceleration axh = 0.7 "Upper acceleration limit";
      parameter SI.Acceleration axl = 0.6 "Lower acceleration limit";
      parameter Real E_factor = 0.9;
      parameter SI.Torque Tsplit = 700;
      parameter Real k = 1 "Boost pressure coefficient";
      Real perc_throttle;
      Real Aped "Accelerator pedal position [%]";
      SI.Velocity Vx "velocity of the vehicle m/s";
      //SI.Torque torque(start = 0) "engine torque";
      SI.Torque Te;
      SI.Torque max_torque;
      SI.Torque max_torque_new;
      SI.Torque min_torque;
      SI.Torque Tbase;
      SI.Torque Tdynreq;
      SI.Torque Ttop;
      SI.Torque T_e;
      SI.Torque torque;
      SI.Force Fx "Force to compute ax";
      SI.AngularVelocity w_engine(start = w_eng_idl) "engine speed in rad/s";
      SI.AngularVelocity rpm_engine "engine speed in rpm";
      Modelica.Blocks.Tables.CombiTable2D tab_torque(table = [0,41.8879,62.8319,73.3038,83.7758,94.2478,104.7198,115.1917,125.6637,136.1357,146.6077,157.0796,167.5516,178.0236,188.4956,198.9675,209.4395;
                                                              0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
                                                              0.1,0,153,171,205,280,340,340,340,340,340,325,320,295,275,255,155;
                                                              0.2,0,299,335,403,553,673,673,673,673,673,643,633,583,543,503,303;
                                                              0.3,0,447,501,603,828,1008,1008,1008,1008,1008,963,948,873,813,753,453;
                                                              0.4,0,600,672,808,1108,1348,1348,1348,1348,1348,1288,1268,1168,1088,1008,608;
                                                              0.5,0,840,930,1100,1475,1775,1775,1775,1775,1775,1700,1675,1550,1450,1350,850;
                                                              0.6,0,1008,1116,1320,1770,2130,2130,2130,2130,2130,2040,2010,1860,1740,1620,1020;
                                                              0.7,0,1176,1302,1540,2065,2485,2485,2485,2485,2485,2380,2345,2170,2030,1890,1190;
                                                              0.8,0,1344,1488,1760,2360,2840,2840,2840,2840,2840,2720,2680,2480,2320,2160,1360;
                                                              0.9,0,1512,1674,1980,2655,3195,3195,3195,3195,3195,3060,3015,2790,2610,2430,1530;
                                                              1.0,0,1680,1860,2200,2950,3550,3550,3550,3550,3550,3400,3350,3100,2900,2700,1700], smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments) "u1:throttle, u2:rps, y:torque";

    equation
      rpm_engine = w_engine * 30 / pi;
      tab_torque.u1 = perc_throttle;
      tab_torque.u2 = w_engine;
      tab_torque.y = max_torque;
      max_torque_new = if der(Vx) > axh then E_factor*max_torque else if der(Vx) < axl then max_torque else max_torque*( (((der(Vx)-axl)*(E_factor-1))/(axh-axl)) + 1);
      min_torque = k3min * rpm_engine ^ 3 + k2min * rpm_engine ^ 2 + k1min * rpm_engine + k0min;
      perc_throttle = max(min(Aped/100, 1), 0.0);
      torque = (((max_torque_new) - min_torque) + min_torque) * (1 - tanh((w_engine - w_eng_max) * 0.01));
      Tbase = min(torque,Tsplit);
      Tdynreq = torque - Tbase;
      der(Ttop) = k*(Tdynreq - Ttop);
      T_e = Tbase + Ttop;
      Te = noEvent(if Vx < 0 then max(0, T_e) else T_e);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end engine_modular_2;

    model transmission_modular_1
      extends Version_7.Driveline.engine_modular_1;
      import SI = Modelica.SIunits;
      constant Real pi = Modelica.Constants.pi;
       // transmission parameters
      parameter Real i_tm[12] = {14.356,11.7239, 9.0363, 7.0941, 5.5382, 4.3478, 3.4363,  2.6977, 2.0783, 1.6316, 1.2738, 1}
        "gear ratios from 1st up to 12th gear";
      parameter Real i_final = 3.46 "gear ratio final gear";
      parameter Real eff_tr = 0.90 "transmission efficiency";
      parameter Real upshift[12] = {2.31,3,3.81,4.86,6.25,8,10.02,12.8,16.65,21.2,27.1,1000000};
      parameter Real downshift[12] = {-1000000,2.2,2.28,3.5,4.5,6,7.5,9.4,12,15.5,20,25};
      parameter SI.Torque max_drive_torque = 12e5;
      parameter Real dds=5 "damping factor for drive shaft flexibility ";
      parameter Integer st_agear;
      Integer agear(start = st_agear) " the automatic gear";
      Real i_T(start = 0) "total transmission ratio";
      Real r_gear;
      Real Je;
      SI.Torque Tc;
      SI.Torque Tt;
      SI.Torque Tp;
      SI.Torque Tf;
      SI.Torque Td;
      SI.Torque Tw;
      SI.AngularVelocity w_c;
      SI.AngularVelocity w_t;
      SI.AngularVelocity w_p;
      SI.AngularVelocity w_f;
      SI.AngularVelocity w_w;
      SI.Torque M_wheel_1;
      SI.Torque M_wheel_2;
      SI.AngularVelocity w_wheel_1;
      SI.AngularVelocity w_wheel_2;
      SI.AngularVelocity w_mean_wheels;
      SI.Torque rev_torque;

      output Real r_c;
    equation

      i_T = i_final * i_tm[agear];
      when Vx > upshift[pre(agear)] then
        agear = pre(agear)+1;
      elsewhen Vx < downshift[pre(agear)] then
        agear = pre(agear)-1;
      end when;

      Je*der(w_engine) = Te- Tc; // Torque balance according to Newtons law
      w_engine = min(w_eng_max,max(w_c,w_eng_idl));

      Tc = Tt;

      w_c = w_t;// the speed of the clutch would be reduced by the gear ratio whcih gives the transmission speed
      Tp = Tt*i_tm[agear];// The torque leaving the

      w_t/i_tm[agear] = w_p; // the speed coming out of the tranmission will be the speed of the propeller shaft

      Tp = Tf;// torque that is entering the final drive is the torque on the propeller shaft

      w_p = w_f;// the speed after the final drive is w_f which is the propeller shaft speed w_p reduced by the final drive ratio
      eff_tr*Tf*i_final = Td;// the torque that comes into the final drive is Tf and is multiplied by i_final to give the Td which is leaving the final drive sent out on to the drive shaft


      Td = Tw; // the drive shaft will be torque that will be exerted on the torque without driveshaft flexibiltiy

      w_f/i_final = max(w_w,0.1); // the speed from the final drive is wheel speed
      w_w = w_mean_wheels;
      w_mean_wheels = (w_wheel_1+w_wheel_2)/2;
      rev_torque = -1*r_c*max(Tt,0)*i_tm[6]*i_final;
      r_c = if r_gear > 0 then 1 else 0;
      M_wheel_2 = max(Tw,0);//fix for - ve velocity on braking
      M_wheel_1 = 0;

        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end transmission_modular_1;

    model transmission_modular_2
      extends Version_7.Driveline.engine_modular_2;
      import SI = Modelica.SIunits;
      constant Real pi = Modelica.Constants.pi;
       // transmission parameters
      parameter Real i_tm[12] = {14.356,11.7239, 9.0363, 7.0941, 5.5382, 4.3478, 3.4363,  2.6977, 2.0783, 1.6316, 1.2738, 1}
        "gear ratios from 1st up to 12th gear";
      parameter Real i_final = 3.46 "gear ratio final gear";
      parameter Real eff_tr = 0.90 "transmission efficiency";
      parameter Real upshift[12] = {2.31,3,3.81,4.86,6.25,8,10.02,12.8,16.65,21.2,27.1,1000000};
      parameter Real downshift[12] = {-1000000,2.2,2.28,3.5,4.5,6,7.5,9.4,12,15.5,20,25};
      parameter SI.Torque max_drive_torque = 12e5;
      parameter Real dds=5 "damping factor for drive shaft flexibility ";
      parameter Integer st_agear;
      Integer agear(start = st_agear) " the automatic gear";
      Real i_T(start = 0) "total transmission ratio";
      Real r_gear;
      Real Je;
      SI.Torque Tc;
      SI.Torque Tt;
      SI.Torque Tp;
      SI.Torque Tf;
      SI.Torque Td;
      SI.Torque Tw;
      SI.AngularVelocity w_c;
      SI.AngularVelocity w_t;
      SI.AngularVelocity w_p;
      SI.AngularVelocity w_f;
      SI.AngularVelocity w_w;

      SI.Torque M_wheel_1;
      SI.Torque M_wheel_2;
      SI.AngularVelocity w_wheel_1;
      SI.AngularVelocity w_wheel_2;
      SI.AngularVelocity w_mean_wheels;
      SI.Torque rev_torque;

      output Real r_c;
    equation

      i_T = i_final * i_tm[agear];
      when Vx > upshift[pre(agear)] then
        agear = pre(agear)+1;
      elsewhen Vx < downshift[pre(agear)] then
        agear = pre(agear)-1;
      end when;

      Je*der(w_engine) = Te- Tc; // Torque balance according to Newtons law
      w_engine = min(w_eng_max,max(w_c,w_eng_idl));

      Tc = Tt;

      w_c = w_t;// the speed of the clutch would be reduced by the gear ratio whcih gives the transmission speed
      Tp = Tt*i_tm[agear];// The torque leaving the

      w_t/i_tm[agear] = w_p; // the speed coming out of the tranmission will be the speed of the propeller shaft

      Tp = Tf;// torque that is entering the final drive is the torque on the propeller shaft

      w_p = w_f;// the speed after the final drive is w_f which is the propeller shaft speed w_p reduced by the final drive ratio
      eff_tr*Tf*i_final = Td;// the torque that comes into the final drive is Tf and is multiplied by i_final to give the Td which is leaving the final drive sent out on to the drive shaft

      //Td = Tw + dds*(w_f - w_w);
    //   der(Td) = Td- Tw+dds*(w_f-w_w);

      Td = Tw; // the drive shaft will be torque that will be exerted on the torque without driveshaft flexibiltiy

      w_f/i_final = w_w; // the speed from the final drive is wheel speed
      w_w = w_mean_wheels;
      w_mean_wheels = (w_wheel_1+w_wheel_2)/2;
      rev_torque = -1*r_c*max(Tt,0)*i_tm[6]*i_final;
      r_c = if r_gear > 0 then 1 else 0;
      M_wheel_2 = max(Tw,0);//fix for - ve velocity on braking
      M_wheel_1 = 0;

        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end transmission_modular_2;

    model engine_expt
      engine_modular_1 engine;
    equation
      engine.Aped = if time < 2 then 0 else if time < 20 then 100 else 0;
      engine.w_engine = if time < 2 then 0 else if time < 20 then 200*(time -2)/18 else 0;
      engine.Fx = 0;

      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end engine_expt;

    model transmission_modular_3
      extends Version_7.Driveline.engine_modular_3;
      import SI = Modelica.SIunits;
      constant Real pi = Modelica.Constants.pi;
       // transmission parameters
      parameter Real i_tm[12] = {14.356,11.7239, 9.0363, 7.0941, 5.5382, 4.3478, 3.4363,  2.6977, 2.0783, 1.6316, 1.2738, 1}
        "gear ratios from 1st up to 12th gear";
      parameter Real i_final = 3.46 "gear ratio final gear";
      parameter Real eff_tr = 0.90 "transmission efficiency";
      parameter Real upshift[12] = {2.31,3,3.81,4.86,6.25,8,10.02,12.8,16.65,21.2,27.1,1000000};
      parameter Real downshift[12] = {-1000000,2.2,2.28,3.5,4.5,6,7.5,9.4,12,15.5,20,25};
      parameter SI.Torque max_drive_torque = 12e5;
      parameter Real dds=5 "damping factor for drive shaft flexibility ";
      parameter Integer st_agear;
      Integer agear(start = st_agear) " the automatic gear";
      Real i_T(start = 0) "total transmission ratio";
      Real r_gear;
      Real Je;
      SI.Torque Tc;
      SI.Torque Tt;
      SI.Torque Tp;
      SI.Torque Tf;
      SI.Torque Td;
      SI.Torque Tw;
      SI.AngularVelocity w_c;
      SI.AngularVelocity w_t;
      SI.AngularVelocity w_p;
      SI.AngularVelocity w_f;
      SI.AngularVelocity w_w;

      SI.Torque M_wheel_1;
      SI.Torque M_wheel_2;
      SI.AngularVelocity w_wheel_1;
      SI.AngularVelocity w_wheel_2;
      SI.AngularVelocity w_mean_wheels;
      SI.Torque rev_torque;

      output Real r_c;
    equation

      i_T = i_final * i_tm[agear];
      when Vx > upshift[pre(agear)] then
        agear = pre(agear)+1;
      elsewhen Vx < downshift[pre(agear)] then
        agear = pre(agear)-1;
      end when;

      Je*der(w_engine) = Te- Tc; // Torque balance according to Newtons law
      w_engine = min(w_eng_max,max(w_c,w_eng_idl));

      Tc = Tt;

      w_c = w_t;// the speed of the clutch would be reduced by the gear ratio whcih gives the transmission speed
      Tp = Tt*i_tm[agear];// The torque leaving the

      w_t/i_tm[agear] = w_p; // the speed coming out of the tranmission will be the speed of the propeller shaft

      Tp = Tf;// torque that is entering the final drive is the torque on the propeller shaft

      w_p = w_f;// the speed after the final drive is w_f which is the propeller shaft speed w_p reduced by the final drive ratio
      eff_tr*Tf*i_final = Td;// the torque that comes into the final drive is Tf and is multiplied by i_final to give the Td which is leaving the final drive sent out on to the drive shaft

      //Td = Tw + dds*(w_f - w_w);
    //   der(Td) = Td- Tw+dds*(w_f-w_w);

      Td = Tw; // the drive shaft will be torque that will be exerted on the torque without driveshaft flexibiltiy

      w_f/i_final = w_w; // the speed from the final drive is wheel speed
      w_w = w_mean_wheels;
      w_mean_wheels = (w_wheel_1+w_wheel_2)/2;
      rev_torque = -1*r_c*max(Tt,0)*i_tm[6]*i_final;
      r_c = if r_gear > 0 then 1 else 0;
      M_wheel_2 = max(Tw,0);//fix for - ve velocity on braking
      M_wheel_1 = 0;

        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end transmission_modular_3;

    model engine_modular_3
      import SI = Modelica.SIunits;
      constant Real pi = Modelica.Constants.pi;
      parameter  SI.Mass m;

      //Engine coefficients
      parameter Real k3max = -1.246e-06;
      parameter Real k2max = 1.471e-04;
      parameter Real k1max = 6.5110;
      parameter Real k0max = -2.2287e+03;
      parameter Real k3min = -1.59e-8;
      parameter Real k2min = 8.7520e-5;
      parameter Real k1min = -0.197569;
      parameter Real k0min = -4.22681;
      parameter SI.AngularVelocity w_eng_max = 2000*pi/30
        "maximum engine rotation";
      parameter SI.AngularVelocity w_eng_idl = 600*pi/30 " engine idle speed";
      parameter SI.Acceleration axh = 0.7 "Upper acceleration limit";
      parameter SI.Acceleration axl = 0.6 "Lower acceleration limit";
      parameter Real E_factor = 0.4;
      parameter SI.Torque Tsplit = 700;
      parameter Real k = 1 "Boost pressure coefficient";
      Real perc_throttle;
      Real Aped "Accelerator pedal position [%]";
      SI.Velocity Vx "velocity of the vehicle m/s";
      SI.Torque torque(start = 0) "engine torque";
      SI.Torque Te;
      SI.Torque max_torque;
      SI.Torque Ta;
      SI.Torque min_torque;
      SI.Torque Tbase;
      SI.Torque Tdynreq;
      SI.Torque Ttop;
      SI.Torque T_e;
      SI.Torque Talim;
      SI.Force Fx "Force to compute ax";
      SI.AngularVelocity w_engine(start = w_eng_idl) "engine speed in rad/s";
      SI.AngularVelocity rpm_engine "engine speed in rpm";
      Modelica.Blocks.Tables.CombiTable1Ds tab_torque(table = [0,0;
                                                                 62.8318,1660;
                                                                 73.3038,1880;
                                                                 83.775,2240;
                                                                 94.247,2900;
                                                                 104.719,3550;
                                                                 115.191,3550;
                                                                 125.663,3550;
                                                                 136.135,3550;
                                                                 146.607,3550;
                                                                 157.079,3470;
                                                                 167.551,3310;
                                                                 178.023,3120;
                                                                 188.495,2880;
                                                                 198.967,2660;
                                                                 209.439,1680;
                                                                 219.911,0],smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments);
    equation
      rpm_engine = w_engine * 30 / pi;
      tab_torque.u = w_engine;
      tab_torque.y[1] = max_torque;
      min_torque = k3min * rpm_engine ^ 3 + k2min * rpm_engine ^ 2 + k1min * rpm_engine + k0min;
      perc_throttle = max(min(Aped/100, 1), 0.0);
      Talim = min(max_torque,(max_torque-Ta));
      torque = (perc_throttle * ((Talim) - min_torque) + min_torque) * (1 - tanh((w_engine - w_eng_max) * 0.01))/ 2;
      Tbase = min(torque,Tsplit);
      Tdynreq = torque - Tbase;
      der(Ttop) = k*(Tdynreq - Ttop);
      T_e = Tbase + Ttop;
      Te = noEvent(if Vx < 0 then max(0, T_e) else T_e);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end engine_modular_3;
  end Driveline;

  package body_and_wheel

    model chassis
      constant Real g=Modelica.Constants.g_n;
      parameter Real m=9481;
      //mass of the vehicle[kg]
      parameter Real a=1.68;
      //distance from CG to front axel [m]
      parameter Real b=1.715;
      //distance from CG to rear axel [m]
      parameter Real Izz=41340; //20490-used 1st,33795-2nd,(ay 3.2(1st),2.9(2nd),2.7(current) (2.4 with updated wheelbase))
      // Mass moment of inertia [kgm^2]
      parameter Real i_s=16;
      // steering ratio
      parameter Real Iw=1;
      // Wheel Inertia[kgm^2]
      parameter Real rw=0.5;
      //Wheel radius [m]
      parameter Real Tb=25000;
      //Braking torque[Nm]
      parameter Real m_f=m*(b/(a + b));
      parameter Real m_r=m*(a/(a + b));
      parameter Real Fzf=m*g*(b/(a + b));
      // Front vertical load
      parameter Real Fzr=m*g*(a/(a + b));
      //Rear vertical load
      parameter Real vx0=0.0;
      //initial longitudinal velocity [m/s]
      parameter Real Fxmax=3000000;
      parameter Real Pmax=29000;
      parameter Real f_r=0.0164 "Rolling resistance coefficient";
      parameter Real rho=1.225 "air density";
      parameter Real Cd=0.7;
      parameter Real ac=10;
      parameter Real cx = 20;
      parameter Real mu= 0.9;
      parameter Real ratio=3.0;
      parameter Real Je=3  "Engine inertia";

      Real vy(start=0.0) "Vehicle Lateral Velocity [m/sec]";
      Real yaw_angle(start=0);
      Real yaw_velocity(start=0);
      Real ay;
      Real Fx;
      Real Fxf;
      Real Fyr;
      Real Fxr;
      Real Fr(start=0.0);
      Real Fd(start=0.0);
      Real Fg; //force due to gradient
      Real Tbw;
      Real delta;
      Real womega;
      Real swa "Steering angle [rad]";
      Real brakeped "[%] i/p of brake pedal";
      Real theta;//road gradient
      output Real vx(start=vx0) "Vehicle longitudinal velocity";
      output Real x(start=0);
      output Real y(start=0);
      output Real ax;
      output Real Fyf;
      output Real decl(start=0);

    equation
      // Longitudinal Dynamics ---------------
      Fx = Fxf + Fxr + Fd + Fr+Fg*tanh(10 * abs(womega)) * tanh(0.5 * abs(vx));
      (m)*ax = (Fx);
      Fr = -f_r*m*g*min(1, vx);
      Fd = -sign(vx)*0.5*rho*Cd*ac*vx^2;
      Fg = -m*g*sin(theta);
      Tbw = brakeped*0.01*Tb;
      // Lateral Dynamics -------------------
      Izz*der(yaw_velocity) = (a*Fyf - b*Fyr);
      m*(ay) = (Fyf + Fyr);
      der(yaw_angle) = yaw_velocity;
      ay = der(vy) + vx*yaw_velocity;
      ax = der(vx) - vy*yaw_velocity;
      der(x) = vx*cos(yaw_angle) - vy*sin(yaw_angle);
      der(y) = vy*cos(yaw_angle) + vx*sin(yaw_angle);
      decl = if ax > 0 then 0 else ax;
      delta = swa/i_s;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end chassis;

    model wheel
      import SI = Modelica.SIunits;
      constant Real pi = Modelica.Constants.pi;
      parameter Real rw = 0.5 "Radius of wheel";
      parameter Real c=20;
      parameter Real mu=0.9;
      parameter Real vx0=0;
      parameter Real I_tyre = 11;
      SI.Velocity Vx(start = vx0) "Chassis long velocity";
      SI.Velocity Vy(start = 0.0) "Chassis lat velocity";
      SI.Velocity vx "wheel lon velocity";
      SI.Velocity vy "wheel lat velcoity";
      SI.AngularVelocity omega(start = vx0/rw) "wheel angular velocity";
      SI.Force Fxy "Tire Force combined";
      SI.Angle delta;
      SI.Force Fz;
      SI.Force Fxw;
      SI.Force Fyw;
      SI.Force Fxv;
      SI.Force Fyv;
      SI.Torque Drive_torque;
      SI.Torque Brake_torque;
      Real sxy "combined slip";
      Real fsxy "tire coefficient";
      Real sx "longitudinal slip";
      Real sy "Lateral slip";
      Real Je "Engine inertia";
      Real i_T "gear and final drive ratio product";
    equation
      vx = Vx * cos(delta) + Vy * sin(delta);
      vy = (-Vx * sin(delta)) + Vy * cos(delta);
      sx = -(vx - omega * rw) / max(abs(omega * rw), 0.01);
      sy = -vy / max(abs(omega * rw),0.01);
      sxy = sqrt((sx^2)+(sy^2));
      fsxy = 2 / pi * atan(c * (2 / pi) * sxy);
      Fxy =  mu * Fz * fsxy;
      Fxw = (rw*omega - vx)*Fxy/max((sqrt((rw*omega - vx)^2 + vy^2)),0.01);
      Fyw = -vy*Fxy/max((sqrt((rw*omega - vx)^2 + vy^2)),0.01);
      (I_tyre) * der(omega) = Drive_torque - tanh(10 * omega) * tanh(0.5 * abs(vx)) * Brake_torque - Fxw * rw;
      Fxv = cos(delta)*Fxw - sin(delta)*Fyw;
      Fyv = sin(delta)*Fxw +cos(delta)*Fyw;
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end wheel;
  end body_and_wheel;
  annotation (uses(Modelica(version="3.2.2")));
end Version_7;
