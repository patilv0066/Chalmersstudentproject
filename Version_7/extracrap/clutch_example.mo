within ;
package clutch_example

  model Friction "Drive train with clutch and brake"
    import Modelica.Constants.pi;
    extends Modelica.Icons.Example;
    parameter Modelica.SIunits.Time startTime=0.5 "Start time of step";
    output Modelica.SIunits.Torque tMotor=torque.tau
      "Driving torque of inertia3";
    output Modelica.SIunits.Torque tClutch=clutch.tau
      "Friction torque of clutch";
    output Modelica.SIunits.Torque tBrake=brake.tau "Friction torque of brake";
    output Modelica.SIunits.Torque tSpring=spring.tau "Spring torque";

    Modelica.Mechanics.Rotational.Sources.Torque torque(useSupport=true)
      annotation (Placement(transformation(extent={{-90,-10},{-70,10}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia3(
      J=1,
      phi(
        start=0,
        fixed=true,
        displayUnit="deg"),
      w(start=100,
        fixed=true,
        displayUnit="rad/s"))
      annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
    Modelica.Mechanics.Rotational.Components.Clutch clutch(fn_max=160)
      annotation (Placement(transformation(extent={{-30,-10},{-10,10}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia2(
      J=0.05,
      phi(start=0, fixed=true),
      w(start=90, fixed=true))
      annotation (Placement(transformation(extent={{0,-10},{20,10}})));
    Modelica.Mechanics.Rotational.Components.SpringDamper spring(c=160, d=1)
      annotation (Placement(transformation(extent={{30,-10},{50,10}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia1(
      J=1,
      phi(start=0, fixed=true),
      w(start=90, fixed=true))
      annotation (Placement(transformation(extent={{90,-10},{110,10}})));
    Modelica.Mechanics.Rotational.Components.Brake brake(fn_max=1600,
        useSupport=true)
      annotation (Placement(transformation(extent={{60,-10},{80,10}})));
    Modelica.Blocks.Sources.Constant const(k=1) annotation (Placement(
          transformation(
          origin={-25,35},
          extent={{-5,-5},{15,15}},
          rotation=270)));
    Modelica.Blocks.Sources.Step step(startTime=startTime) annotation (
        Placement(transformation(
          origin={65,35},
          extent={{-5,-5},{15,15}},
          rotation=270)));
    Modelica.Blocks.Sources.Step step2(
      height=-1,
      offset=1,
      startTime=startTime) annotation (Placement(transformation(extent={{-160,
              -30},{-140,-10}})));
    Modelica.Blocks.Sources.Sine sine(amplitude=200, freqHz=50/pi)
      annotation (Placement(transformation(extent={{-160,10},{-140,30}})));
    Modelica.Blocks.Math.Product product annotation (Placement(transformation(
            extent={{-120,-10},{-100,10}})));
    Modelica.Mechanics.Rotational.Components.Fixed fixed
      annotation (Placement(transformation(extent={{-10,-30},{10,-10}})));
  equation
    connect(torque.flange, inertia3.flange_a)
      annotation (Line(points={{-70,0},{-70,0},{-60,0}}));
    connect(inertia3.flange_b, clutch.flange_a)
      annotation (Line(points={{-40,0},{-36,0},{-30,0}}));
    connect(clutch.flange_b, inertia2.flange_a)
      annotation (Line(points={{-10,0},{-6,0},{0,0}}));
    connect(inertia2.flange_b, spring.flange_a)
      annotation (Line(points={{20,0},{30,0}}));
    connect(spring.flange_b, brake.flange_a)
      annotation (Line(points={{50,0},{60,0}}));
    connect(brake.flange_b, inertia1.flange_a)
      annotation (Line(points={{80,0},{80,0},{90,0}}));
    connect(sine.y, product.u1) annotation (Line(points={{-139,20},{-130,20},
            {-130,6},{-122,6}}, color={0,0,127}));
    connect(step2.y, product.u2) annotation (Line(points={{-139,-20},{-130,-20},
            {-130,-6},{-126,-6},{-122,-6}}, color={0,0,127}));
    connect(product.y, torque.tau)
      annotation (Line(points={{-99,0},{-99,0},{-92,0}}, color={0,0,127}));
    connect(const.y, clutch.f_normalized) annotation (Line(points={{-20,19},{
            -20,11}},             color={0,0,127}));
    connect(step.y, brake.f_normalized)
      annotation (Line(points={{70,19},{70,16},{70,11}}, color={0,0,127}));
    connect(torque.support, fixed.flange)
      annotation (Line(points={{-80,-10},{-80,-20},{0,-20}}));
    connect(brake.support, fixed.flange)
      annotation (Line(points={{70,-10},{70,-20},{0,-20}}));
    annotation (Documentation(info="<html>
<p>This drive train contains a frictional <b>clutch</b> and a <b>brake</b>.
Simulate the system for 1 second using the following initial
values (defined already in the model):</p>
<pre>   inertia1.w =  90 (or brake.w)
   inertia2.w =  90
   inertia3.w = 100
</pre>
<p>Plot the output signals</p>
<pre>   tMotor      Torque of motor
   tClutch     Torque in clutch
   tBrake      Torque in brake
   tSpring     Torque in spring
</pre>
<p>as well as the absolute angular velocities of the three inertia components
(inertia1.w, inertia2.w, inertia3.w).</p>

</html>"),
         experiment(StopTime=3.0, Interval=0.001),
    Diagram(coordinateSystem(extent = {{-170,-100},{120,100}})));
  end Friction;

  model try1
    Modelica.Mechanics.Rotational.Sources.Torque torque(useSupport=true)
      annotation (Placement(transformation(extent={{-56,-2},{-36,18}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia3(
      J=1,
      phi(
        start=0,
        fixed=true,
        displayUnit="deg"),
      w(start=0,
        fixed=true,
        displayUnit="rad/s"))
      annotation (Placement(transformation(extent={{-24,-2},{-4,18}})));
    Modelica.Mechanics.Rotational.Components.Clutch clutch(fn_max=1250)
      annotation (Placement(transformation(extent={{6,-2},{26,18}})));
    Modelica.Mechanics.Rotational.Components.Fixed
                                fixed annotation (Placement(transformation(
            extent={{-56,-34},{-36,-14}})));
    Modelica.Mechanics.Rotational.Components.Inertia inertia1(
      J=1,
      phi(
        start=0,
        fixed=true,
        displayUnit="deg"),
      w(start=0,
        fixed=true,
        displayUnit="rad/s"))
      annotation (Placement(transformation(extent={{32,-2},{52,18}})));
    Modelica.Blocks.Sources.Trapezoid trapezoid(
      rising=1,
      falling=1,
      amplitude=250,
      startTime=1,
      width=7,
      period=25)
      annotation (Placement(transformation(extent={{-98,-2},{-78,18}})));
    Modelica.Blocks.Sources.Step step(startTime=10, offset=0)
                                                   annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={14,78})));
    Modelica.Mechanics.Rotational.Sources.Torque torque1(
                                                        useSupport=true)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={74,-20})));
    Modelica.Blocks.Sources.Step step1(startTime=16, height=-275) annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={110,-20})));
    Modelica.Blocks.Sources.Pulse pulse(
      period=25,
      amplitude=250,
      startTime=1)
      annotation (Placement(transformation(extent={{-98,-34},{-78,-14}})));
    Modelica.Blocks.Sources.Pulse pulse1(
      startTime=16,
      amplitude=-185,
      period=15) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={106,-88})));
    Modelica.Blocks.Sources.Ramp ramp(
      duration=5,
      startTime=16,
      height=-350) annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={108,-52})));
  equation
    connect(torque.flange, inertia3.flange_a)
      annotation (Line(points={{-36,8},{-30,8},{-24,8}},
                                              color={0,0,0}));
    connect(clutch.flange_a, inertia3.flange_b)
      annotation (Line(points={{6,8},{-4,8}},  color={0,0,0}));
    connect(fixed.flange, torque.support)
      annotation (Line(points={{-46,-24},{-46,-2}},           color={0,0,0}));
    connect(clutch.flange_b, inertia1.flange_a)
      annotation (Line(points={{26,8},{32,8}}, color={0,0,0}));
    connect(inertia1.flange_b, torque1.flange) annotation (Line(points={{52,8},
            {56,8},{56,-20},{64,-20}}, color={0,0,0}));
    connect(torque1.support, fixed.flange) annotation (Line(points={{74,-10},{
            14,-10},{14,-24},{-46,-24}}, color={0,0,0}));
    connect(step.y, clutch.f_normalized) annotation (Line(points={{14,67},{14,
            44},{14,19},{16,19}}, color={0,0,127}));
    connect(torque1.tau, step1.y)
      annotation (Line(points={{86,-20},{99,-20}}, color={0,0,127}));
    connect(pulse.y, torque.tau) annotation (Line(points={{-77,-24},{-68,-24},{
            -68,8},{-58,8}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end try1;

  package modified_clutch

    model Clutch_m "Clutch based on Coulomb friction"
      constant Real pi = Modelica.Constants.pi;
      extends Modelica.Mechanics.Rotational.Icons.Clutch;
    //   extends
    //     Modelica.Mechanics.Rotational.Interfaces.PartialCompliantWithRelativeStates;

      parameter Real mue_pos[:, 2]=[0, 0.4]
        "[w,mue] positive sliding friction coefficient (w_rel>=0)";
      parameter Real peak(final min=1) = 1
        "peak*mue_pos[1,2] = maximum value of mue for w_rel==0";
      parameter Real cgeo(final min=0) = 3.2
        "Geometry constant containing friction distribution assumption";
      parameter Modelica.SIunits.Force fn_max(final min=0, start=1)
        "Maximum normal force";

      extends Modelica.Mechanics.Rotational.Interfaces.PartialFriction;
      extends
        Modelica.Thermal.HeatTransfer.Interfaces.PartialElementaryConditionalHeatPortWithoutT;
      Real w_rel;
      Real wl(start =  600*pi/30);
      Real wr(start = 0);
      //Real a_rel;
      Real tau;
      Real mue0 "Friction coefficient for w=0 and forward sliding";
      Modelica.SIunits.Force fn "Normal force (fn=fn_max*f_normalized)";
      Real f_normalized;
    //   Modelica.Blocks.Interfaces.RealInput f_normalized
    //     "Normalized force signal 0..1 (normal force = fn_max*f_normalized; clutch is engaged if > 0)"
    //

    equation
      // Constant auxiliary variable
      mue0 = Modelica.Math.Vectors.interpolate(mue_pos[:,1], mue_pos[:,2], 0, 1);

      // Relative quantities
      w_rel = wr-wl;
      w_relfric = w_rel;
      a_relfric = der(w_rel);

      // Normal force and friction torque for w_rel=0
      fn = fn_max*f_normalized;
      free = fn <= 0;
      tau0 = mue0*cgeo*fn;
      tau0_max = peak*tau0;

      // Friction torque
      tau = if locked then sa*unitTorque else if free then 0 else cgeo*fn*(
        if startForward then
          Modelica.Math.Vectors.interpolate(mue_pos[:,1], mue_pos[:,2], w_rel, 1)
        else if startBackward then
          -Modelica.Math.Vectors.interpolate(mue_pos[:,1], mue_pos[:,2], w_rel, 1)
        else if pre(mode) == Forward then
          Modelica.Math.Vectors.interpolate(mue_pos[:,1], mue_pos[:,2], w_rel, 1)
        else
          -Modelica.Math.Vectors.interpolate(mue_pos[:,1], mue_pos[:,2], -w_rel, 1));
      lossPower = tau*w_relfric;
      annotation (Icon(
          coordinateSystem(preserveAspectRatio=true,
            extent={{-100,-100},{100,100}}),
            graphics={
          Text(extent={{-150,-110},{150,-70}},
            textString="%name",
            lineColor={0,0,255}),
          Line(visible=useHeatPort,
            points={{-100,-100},{-100,-40},{0,-40}},
            color={191,0,0},
            pattern=LinePattern.Dot)}), Documentation(info="<html>
<p>
This component models a <b>clutch</b>, i.e., a component with
two flanges where friction is present between the two flanges
and these flanges are pressed together via a normal force.
The normal force fn has to be provided as input signal f_normalized in a normalized form
(0 &le; f_normalized &le; 1),
fn = fn_max*f_normalized, where fn_max has to be provided as parameter. Friction in the
clutch is modelled in the following way:
</p>
<p>
When the relative angular velocity is not zero, the friction torque is a
function of the velocity dependent friction coefficient  mue(w_rel) , of
the normal force \"fn\", and of a geometry constant \"cgeo\" which takes into
account the geometry of the device and the assumptions on the friction
distributions:
</p>
<pre>
        frictional_torque = <b>cgeo</b> * <b>mue</b>(w_rel) * <b>fn</b>
</pre>
<p>
   Typical values of coefficients of friction:
</p>
<pre>
      dry operation   :  <b>mue</b> = 0.2 .. 0.4
      operating in oil:  <b>mue</b> = 0.05 .. 0.1
</pre>
<p>
   When plates are pressed together, where  <b>ri</b>  is the inner radius,
   <b>ro</b> is the outer radius and <b>N</b> is the number of friction interfaces,
   the geometry constant is calculated in the following way under the
   assumption of a uniform rate of wear at the interfaces:
</p>
<pre>
         <b>cgeo</b> = <b>N</b>*(<b>r0</b> + <b>ri</b>)/2
</pre>
<p>
    The positive part of the friction characteristic <b>mue</b>(w_rel),
    w_rel >= 0, is defined via table mue_pos (first column = w_rel,
    second column = mue). Currently, only linear interpolation in
    the table is supported.
</p>
<p>
   When the relative angular velocity becomes zero, the elements
   connected by the friction element become stuck, i.e., the relative
   angle remains constant. In this phase the friction torque is
   calculated from a torque balance due to the requirement, that
   the relative acceleration shall be zero.  The elements begin
   to slide when the friction torque exceeds a threshold value,
   called the  maximum static friction torque, computed via:
</p>
<pre>
       frictional_torque = <b>peak</b> * <b>cgeo</b> * <b>mue</b>(w_rel=0) * <b>fn</b>   (<b>peak</b> >= 1)
</pre>
<p>
This procedure is implemented in a \"clean\" way by state events and
leads to continuous/discrete systems of equations if friction elements
are dynamically coupled. The method is described in
(see also a short sketch in <a href=\"modelica://Modelica.Mechanics.Rotational.UsersGuide.ModelingOfFriction\">UsersGuide.ModelingOfFriction</a>):
</p>
<dl>
<dt>Otter M., Elmqvist H., and Mattsson S.E. (1999):</dt>
<dd><b>Hybrid Modeling in Modelica based on the Synchronous
    Data Flow Principle</b>. CACSD'99, Aug. 22.-26, Hawaii.</dd>
</dl>
<p>
More precise friction models take into account the elasticity of the
material when the two elements are \"stuck\", as well as other effects,
like hysteresis. This has the advantage that the friction element can
be completely described by a differential equation without events. The
drawback is that the system becomes stiff (about 10-20 times slower
simulation) and that more material constants have to be supplied which
requires more sophisticated identification. For more details, see the
following references, especially (Armstrong and Canudas de Witt 1996):
</p>
<dl>
<dt>Armstrong B. (1991):</dt>
<dd><b>Control of Machines with Friction</b>. Kluwer Academic
    Press, Boston MA.<br></dd>
<dt>Armstrong B., and Canudas de Wit C. (1996):</dt>
<dd><b>Friction Modeling and Compensation.</b>
    The Control Handbook, edited by W.S.Levine, CRC Press,
    pp. 1369-1382.<br></dd>
<dt>Canudas de Wit C., Olsson H., Astroem K.J., and Lischinsky P. (1995):</dt>
<dd><b>A new model for control of systems with friction.</b>
    IEEE Transactions on Automatic Control, Vol. 40, No. 3, pp. 419-425.</dd>
</dl>

<p>
See also the discussion
<a href=\"modelica://Modelica.Mechanics.Rotational.UsersGuide.StateSelection\">State Selection</a>
in the User's Guide of the Rotational library.
</p>
</html>"));
    end Clutch_m;
  end modified_clutch;

  model simple_clutch
    // works the same as the try1 example (with the gear changes commeneted)
    extends clutch_example.modified_clutch.Clutch_m(fn_max = 1250);
    Real T;
    Real w_e(start = 0);
    //Real T_t;
    Real T_ext;
    Real ww(start = 0);
    //Real wt;
    Real Vx;
    //Real check(start = 1);
    //Integer agear(start = 1);
    parameter Real i_tm[12] = {14.356,11.7239, 9.0363, 7.0941, 5.5382, 4.3478, 3.4363,  2.6977, 2.0783, 1.6316, 1.2738, 1};
    parameter Real Je = 1;
    parameter Real Jw = 1;
    parameter Real upshift[12] = {2.31,3,3.81,4.86,6.25,8,10.02,12.8,16.65,21.2,27.1,1000000};
    parameter Real downshift[12] = {-1000000,2.2,2.28,3.5,4.5,6,7.5,9.4,12,15.5,20,25};
  equation
    Vx = if time < 1 then 0 else if time < 2 then 10*(time - 1)/1 else if time < 3 then (10 - 10*(time - 2)) else 0;
    f_normalized = if time < 1 then 0 else if time < 10 then 0 else 1;
    T = if time < 1 then 0  else if time < 13.5 then 250 else  0;
    Je*der(w_e) = T + tau;
    w_e = wl;
    Je*der(wr)=T_ext - tau;
    //T_t = tau;
    ww = wr;
    //T_t = T_ext;
  //   when Vx > upshift[pre(agear)] then
  //     check = (pre(check) + 1)/(1);
  //     agear = pre(agear)+1;
  //   elsewhen Vx < downshift[pre(agear)] then
  //     agear = pre(agear)-1;
  //     check = (pre(check)-1)/1;
  //   end when;
  //   T_t*i_tm[agear] = T_ext;
  //   wt/i_tm[agear] = ww;
    T_ext = if time < 1 then 0  else if time < 16 then 0 else  -275;
       annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end simple_clutch;

  model simple_clutch_2
    extends clutch_example.modified_clutch.Clutch_m(fn_max = 1500);
    extends clutch_example.engine_modular;
    //extends clutch_example.wheel;
    Real T;
    Real T_t;
    Real T_ext;
    Real ww(start = 0);
    Real wt;
    //Real check(start = 1);
    //Integer agear(start = 1);
    parameter Real i_tm[12] = {14.356,11.7239, 9.0363, 7.0941, 5.5382, 4.3478, 3.4363,  2.6977, 2.0783, 1.6316, 1.2738, 1};
    parameter Real Je = 1;
    parameter Real Jw = 1;
    parameter Real upshift[12] = {2.31,3,3.81,4.86,6.25,8,10.02,12.8,16.65,21.2,27.1,1000000};
    parameter Real downshift[12] = {-1000000,2.2,2.28,3.5,4.5,6,7.5,9.4,12,15.5,20,25};
    parameter Real m = 1000;
  equation
    //m*der(Vx) = Fxw -1.0125*(abs(Vx))^2;
    //Vy = 0;
    //delta = 0;
    //Fz = m*9.81;
    //Vx = if time < 1 then 0 else if time < 2 then 10*(time - 1)/1 else if time < 3 then (10 - 10*(time - 2)) else 0;
    f_normalized = if time < 1 then 0 else if time < 2 then 0 else if time < 5 then 0 else 0;
    Aped = if time < 2 then 0 else if time < 5 then 50 else 0;
    T = Te;
    Je*der(w_engine) = T + tau;
    w_engine = wl;
    Je*der(wr)=T_t - tau;
    //T_t = tau;
    wt = wr;
  //    when Vx > upshift[pre(agear)] then
  //      agear = pre(agear)+1;
  //    elsewhen Vx < downshift[pre(agear)] then
  //      agear = pre(agear)-1;
  //    end when;
    T_t    *1.5             = T_ext;
        /*i_tm[agear]*/
    wt    /1.5            = ww;
    T_ext = 0;
      /*i_tm[agear]*/
    //Brake_torque =  if time < 1 then 0  else if time < 10 then 0 else if time < 15 then 0 else 0;
       annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end simple_clutch_2;

  model engine_modular
    import SI = Modelica.SIunits;
    constant Real pi = Modelica.Constants.pi;
    //parameter  SI.Mass m;

    //Engine coefficients
    parameter Real k3max = -1.246e-06;
    parameter Real k2max = 1.471e-04;
    parameter Real k1max = 6.5110;
    parameter Real k0max = -2.2287e+03;
    parameter Real k3min = -1.59e-8;
    parameter Real k2min = 8.7520e-5;
    parameter Real k1min = -0.197569;
    parameter Real k0min = -4.22681;
    parameter SI.AngularVelocity w_eng_max = 2300*pi/30
      "maximum engine rotation";
    parameter SI.AngularVelocity w_eng_idl = 600*pi/30 " engine idle speed";
    parameter SI.Acceleration axh = 0.7 "Upper acceleration limit";
    parameter SI.Acceleration axl = 0.6 "Lower acceleration limit";
    parameter Real E_factor = 0.45;
    parameter SI.Torque Tsplit = 700;
    parameter Real k = 1 "Boost pressure coefficient";
    Real perc_throttle;
    Real Aped "Accelerator pedal position [%]";
    //SI.Velocity Vx "velocity of the vehicle m/s";
    SI.Torque torque(start = 0) "engine torque";
    SI.Torque Te;
    SI.Torque max_torque;
    //SI.Torque max_torque_alim;
    SI.Torque min_torque;
    SI.Torque Tbase;
    SI.Torque Tdynreq;
    SI.Torque Ttop;
    SI.Torque T_e;
    //SI.Force Fx "Force to compute ax";
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
    tab_torque.u = min(w_eng_max,max(w_engine,w_eng_idl));
    tab_torque.y[1] = max_torque;
    //max_torque_alim = if der(Vx) > axh then E_factor*max_torque else if der(Vx) < axl then max_torque else max_torque*( (((der(Vx)-axl)*(E_factor-1))/(axh-axl)) + 1);
    min_torque = k3min * rpm_engine ^ 3 + k2min * rpm_engine ^ 2 + k1min * rpm_engine + k0min;
    perc_throttle = max(min(Aped/100, 1), 0.0);
    torque = (perc_throttle * ((max_torque) - min_torque) + min_torque) * (1 - tanh((w_engine - w_eng_max) * 0.01)) - 20;
    Tbase = min(torque,Tsplit);
    Tdynreq = torque - Tbase;
    der(Ttop) = k*(Tdynreq - Ttop);
    T_e = Tbase + Ttop;
    Te = max(T_e,0);
     annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end engine_modular;

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

  model simple_clutch_en
    // works the same as the try1 example (with the gear changes commeneted)
    extends clutch_example.modified_clutch.Clutch_m(fn_max = 8600);
    extends clutch_example.engine_modular;
    Real T;
    Real w_e(start = 0);
    //Real T_t;
    Real T_ext;
    Real ww(start = 0);
    //Real wt;
    Real Vx;
    //Real check(start = 1);
    //Integer agear(start = 1);
    parameter Real i_tm[12] = {14.356,11.7239, 9.0363, 7.0941, 5.5382, 4.3478, 3.4363,  2.6977, 2.0783, 1.6316, 1.2738, 1};
    parameter Real Je = 4;
    parameter Real Jw = 1;
    parameter Real upshift[12] = {2.31,3,3.81,4.86,6.25,8,10.02,12.8,16.65,21.2,27.1,1000000};
    parameter Real downshift[12] = {-1000000,2.2,2.28,3.5,4.5,6,7.5,9.4,12,15.5,20,25};
  equation
    Vx = if time < 1 then 0 else if time < 2 then 10*(time - 1)/1 else if time < 3 then (10 - 10*(time - 2)) else 0;
    f_normalized = if time < 1 then 0 else if time < 13 then 0 else if time < 22 then 1 else 0;
    T = Te;
    w_engine = w_e;
    Aped = if time < 10 then 0 else if time < 15 then 50 else 0;
    Je*der(w_e) = T + tau -25;
    w_e = wl;
    Jw*der(wr)=T_ext - tau;
    ww = wr;
  //   when Vx > upshift[pre(agear)] then
  //     check = (pre(check) + 1)/(1);
  //     agear = pre(agear)+1;
  //   elsewhen Vx < downshift[pre(agear)] then
  //     agear = pre(agear)-1;
  //     check = (pre(check)-1)/1;
  //   end when;
  //   T_t*i_tm[agear] = T_ext;
  //   wt/i_tm[agear] = ww;
    T_ext = if time < 1 then 0  else if time < 16 then 0 else  -150;
       annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end simple_clutch_en;

  model simple_clutch_wh
    // works the same as the try1 example (with the gear changes commeneted)
    extends clutch_example.modified_clutch.Clutch_m(fn_max = 810560/7);
    extends clutch_example.engine_modular;
    extends clutch_example.wheel;
    Real T;
    Real w_e(start = 0);
    Real T_t;
    Real wt;
    Real Vx;
    //Real check(start = 1);
    Integer agear(start = 1);
    parameter Real m = 9814;
    parameter Real i_tm[12] = {14.356,11.7239, 9.0363, 7.0941, 5.5382, 4.3478, 3.4363,  2.6977, 2.0783, 1.6316, 1.2738, 1};
    parameter Real Je = 4;
    parameter Real Jw = 1;
    parameter Real upshift[12] = {2.31,3,3.81,4.86,6.25,8,10.02,12.8,16.65,21.2,27.1,1000000};
    parameter Real downshift[12] = {-1000000,2.2,2.28,3.5,4.5,6,7.5,9.4,12,15.5,20,25};
  equation
    m*der(Vx) = Fxw - 1.0125*(Vx^2);
    Vy = 0;
    delta = 0;
    Fz = m*9.81;
    f_normalized = if time < 1 then 0 else if time < 10 then 0 else if time < 22 then 1 else 0;
    T = Te;
    w_engine = w_e;
    Aped = if time < 10 then 0 else if time < 15 then 50*(time - 10)/5 else 0;
    Je*der(w_e) = T + tau -25;
    w_e = wl;
    Jw*der(wr)=T_t - tau;
    wr = wt;
     when Vx > upshift[pre(agear)] then
       agear = pre(agear)+1;
     elsewhen Vx < downshift[pre(agear)] then
       agear = pre(agear)-1;
     end when;
     T_t*i_tm[agear] = Drive_torque;
     wt/i_tm[agear] = omega;
     Brake_torque = if time < 1 then 0  else if time < 16 then 0 else  150;
       annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end simple_clutch_wh;
  annotation (uses(Modelica(version="3.2.2")));
end clutch_example;
