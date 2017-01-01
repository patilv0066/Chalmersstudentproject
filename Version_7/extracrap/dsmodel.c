/* DSblock model generated by Dymola from Modelica model clutch_example.simple_clutch_2
 Dymola Version 2017 (64-bit), 2016-06-23 translated this at Sat Dec 31 15:48:44 2016

   */

#include <matrixop.h>
/* Declaration of C-structs */
struct DymStruc0;
struct DymStruc0 {
  int  id_0member;
};
DYMOLA_STATIC struct DymStruc0 DymStruc0_construct(int   id_02) {
  struct DymStruc0 dummy_0;
  dummy_0.id_0member = id_02;
  return dummy_0;
}
/* Prototypes for functions used in model */
DYMOLA_STATIC double   Modelica_Blocks_Tables_CombiTable1Ds_getTableValue_M(
  struct DymStruc0 tableID0_0, int  icol0_0, double  u0_0, double  
  tableAvailable0_0);
DYMOLA_STATIC struct DymStruc0  Modelica_Blocks_Types_ExternalCombiTable1D_M(
  const char*  tableName0_0, const char*  fileName0_0, RealArray   table0_0, 
  IntegerArray   columns0_0, int  smoothness0_0, int id_, int alwaysMakeNew_);
DYMOLA_STATIC void Modelica_Blocks_Types_ExternalCombiTable1D_destructor_M(
  void* externalCombiTable1D0_0);
struct Modelica_Math_Vectors_interpolate_struct {
  double   yi0_0_0member;
  int   iNew0_0_0member;
};
DYMOLA_STATIC struct Modelica_Math_Vectors_interpolate_struct Modelica_Math_Vectors_interpolate
  (RealArray   x0_0, RealArray   y0_0, double  xi0_0, int  iLast0_0);
/* Codes used in model */

/* Flattened Modelica model:

function Modelica.Blocks.Tables.CombiTable1Ds.getTableValue
input Modelica.Blocks.Types.ExternalCombiTable1D tableID;
input Integer icol;
discrete input Real u;
discrete input Real tableAvailable "Dummy input to ensure correct sorting of function calls";
discrete output Real y;

external "C" y = ModelicaStandardTables_CombiTable1D_getValue(tableID, icol, u);
annotation(derivative(noDerivative=tableAvailable)=Modelica.Blocks.Tables.CombiTable1Ds.getDerTableValue, Library={"ModelicaMatIO", "ModelicaStandardTables", "zlib"}, LibraryDirectory="C:/Program Files (x86)/Dymola 2017/Modelica/Library/Modelica 3.2.2/Resources/Library");
end Modelica.Blocks.Tables.CombiTable1Ds.getTableValue;

  */
DYMOLA_STATIC double   Modelica_Blocks_Tables_CombiTable1Ds_getTableValue_M(
  struct DymStruc0 tableID0_0, int  icol0_0, double  u0_0, double  
  tableAvailable0_0) {
  PushContext("Modelica.Blocks.Tables.CombiTable1Ds.getTableValue")
  {
    /* Declare outputs and temporaries */
    double   y0_0;
    y0_0=0;
    /* Start of real code */
    {
      extern double (ModelicaStandardTables_CombiTable1D_getValue)(void*, int , 
        double );
      y0_0 = (ModelicaStandardTables_CombiTable1D_getValue)(externalTable_[
        tableID0_0.id_0member].obj_, icol0_0, u0_0);
      }
    /* Output section */
    PopContext()
    return y0_0;
  }}

/* Flattened Modelica model:

function Modelica.Blocks.Types.ExternalCombiTable1D
input String tableName "Table name";
input String fileName "File name";
discrete input Real table[:, :];
input Integer columns[:];
input Modelica.Blocks.Types.Smoothness smoothness;
output Modelica.Blocks.Types.ExternalCombiTable1D externalCombiTable1D;

external "C" externalCombiTable1D = ModelicaStandardTables_CombiTable1D_init(tableName, fileName, table, size(table, 1), size(table, 2), columns, size(columns, 1), smoothness);
annotation(Library={"ModelicaStandardTables"}, LibraryDirectory="C:/Program Files (x86)/Dymola 2017/Modelica/Library/Modelica 3.2.2/Resources/Library");
end Modelica.Blocks.Types.ExternalCombiTable1D;

  */
DYMOLA_STATIC struct DymStruc0  Modelica_Blocks_Types_ExternalCombiTable1D_M(
  const char*  tableName0_0, const char*  fileName0_0, RealArray   table0_0, 
  IntegerArray   columns0_0, int  smoothness0_0, int id_, int alwaysMakeNew_) {
  PushContext("Modelica.Blocks.Types.ExternalCombiTable1D")
  {
    /* Declare outputs and temporaries */
    struct DymStruc0  externalCombiTable1D0_0;
    externalCombiTable1D0_0.id_0member=0;
    /* Start of real code */
    {
      extern void* (ModelicaStandardTables_CombiTable1D_init)(const char* , 
        const char* , double  const *, size_t, size_t, int  const *, size_t, int );
      externalCombiTable1D0_0.id_0member=id_;
      if (externalTable_[id_].destructor_ && externalTable_[id_].obj_ && alwaysMakeNew_) {
      void*x_=externalTable_[id_].obj_;
      externalTable_[id_].obj_=0;
      (*(externalTable_[id_].destructor_))(x_);
      } else if (!externalTable_[id_].destructor_) {externalTable_[id_].obj_=0;}
      externalTable_[id_].destructor_=Modelica_Blocks_Types_ExternalCombiTable1D_destructor_M;

        if (!externalTable_[id_].obj_) externalTable_[id_].obj_ = (
        ModelicaStandardTables_CombiTable1D_init)(tableName0_0, fileName0_0, 
        table0_0.data, table0_0.dims[1-1], table0_0.dims[2-1], columns0_0.data, 
        columns0_0.dims[1-1], smoothness0_0);
      }
    /* Output section */
    PopContext()
    return externalCombiTable1D0_0;
  }}

/* Flattened Modelica model:

function Modelica.Blocks.Types.ExternalCombiTable1D.destructor
input Modelica.Blocks.Types.ExternalCombiTable1D externalCombiTable1D;

external "C" ModelicaStandardTables_CombiTable1D_close(externalCombiTable1D);
annotation(Library={"ModelicaStandardTables"}, LibraryDirectory="C:/Program Files (x86)/Dymola 2017/Modelica/Library/Modelica 3.2.2/Resources/Library");
end Modelica.Blocks.Types.ExternalCombiTable1D.destructor;

  */
DYMOLA_STATIC void Modelica_Blocks_Types_ExternalCombiTable1D_destructor_M(
  void* externalCombiTable1D0_0) {
  PushContext("Modelica.Blocks.Types.ExternalCombiTable1D.destructor")
  {
    /* Declare outputs and temporaries */
    /* Start of real code */
    {
      extern void (ModelicaStandardTables_CombiTable1D_close)(void*);
      (ModelicaStandardTables_CombiTable1D_close)(externalCombiTable1D0_0);
      }
    /* Output section */
    PopContext()
    return ;
  }}

/* Flattened Modelica model:

function Modelica.Math.Vectors.interpolate
discrete input Real x[:] "Abscissa table vector (strict monotonically increasing values required)";
discrete input Real y[size(x, 1)] "Ordinate table vector";
discrete input Real xi "Desired abscissa value";
input Integer iLast(start = 1) "Index used in last search";
discrete output Real yi "Ordinate value corresponding to xi";
output Integer iNew(start = 1) "xi is in the interval x[iNew] <= xi < x[iNew+1]";
protected 
Integer i;
Integer nx(start = size(x, 1));
discrete Real x1;
discrete Real x2;
discrete Real y1;
discrete Real y2;
public 
algorithm 
assert(nx > 0, "The table vectors must have at least 1 entry.");
if (nx == 1) then 
yi := y[1];
else
i := min(max(iLast, 1), nx-1);
if (xi >= x[i]) then 
while i < nx and xi >= x[i] loop
i := i+1;
end while;
i := i-1;
else
while i > 1 and xi < x[i] loop
i := i-1;
end while;
end if;
x1 := x[i];
x2 := x[i+1];
y1 := y[i];
y2 := y[i+1];
assert(x2 > x1, "Abscissa table vector values must be increasing");
yi := y1+(y2-y1)*(xi-x1)/(x2-x1);
iNew := i;
end if;
end Modelica.Math.Vectors.interpolate;

  */
DYMOLA_STATIC struct Modelica_Math_Vectors_interpolate_struct Modelica_Math_Vectors_interpolate
  (RealArray   x0_0, RealArray   y0_0, double  xi0_0, int  iLast0_0) {
  PushContext("Modelica.Math.Vectors.interpolate")
  AssertModelica(DYNSizeSimple(y0_0,1)==DYNSizeSimple(x0_0, 1),"size(y, 1) == size(x, 1)","Dimension check of input to function failed");
  {
    /* Declare outputs and temporaries */
    double   yi0_0;
    int   iNew0_0;
    int   i0_0;
    int   nx0_0;
    double   x10_0;
    double   x20_0;
    double   y10_0;
    double   y20_0;
    yi0_0=0;
    iNew0_0 = 1;
    i0_0=0;
    nx0_0 = DYNSizeSimple(x0_0, 1);
    x10_0=0;
    x20_0=0;
    y10_0=0;
    y20_0=0;
    /* Start of real code */
      AssertModelica(nx0_0 > 0,"nx > 0", "The table vectors must have at least 1 entry.");
      if (nx0_0 == 1) {
        yi0_0 = RealElement( y0_0, (SizeType)(1));
      }
      else{
        i0_0 = IntegerBmin(IntegerBmax(iLast0_0, 1), nx0_0-1);
        if (xi0_0 >= RealElement( x0_0, (SizeType)(i0_0))) {
          while (i0_0 < nx0_0 AND xi0_0 >= RealElement( x0_0, (SizeType)(i0_0))) {
            i0_0 = i0_0+1;
          }
          i0_0 = i0_0-1;
        }
        else{
          while (i0_0 > 1 AND xi0_0 < RealElement( x0_0, (SizeType)(i0_0))) {
            i0_0 = i0_0-1;
          }
        }
        x10_0 = RealElement( x0_0, (SizeType)(i0_0));
        x20_0 = RealElement( x0_0, (SizeType)(i0_0+1));
        y10_0 = RealElement( y0_0, (SizeType)(i0_0));
        y20_0 = RealElement( y0_0, (SizeType)(i0_0+1));
        AssertModelica(x20_0 > x10_0,"x2 > x1", "Abscissa table vector values must be increasing");
        yi0_0 = y10_0+divmacro((y20_0-y10_0)*(xi0_0-x10_0),"(y2-y1)*(xi-x1)",
          x20_0-x10_0,"x2-x1");
        iNew0_0 = i0_0;
      }
    /* Output section */
    PopContext()
    {
      struct Modelica_Math_Vectors_interpolate_struct out_;
      out_.yi0_0_0member = yi0_0;
      out_.iNew0_0_0member = iNew0_0;
      return out_;
    }
  }}
/* DSblock C-code: */

#define NX_    3
#define NX2_   0
#define NU_    0
#define NY_    0
#define NW_    49
#define NP_    93
#define NPS_   0
#define MAXAuxStr_   2
#define MAXAuxStrLen_   500
#define NHash1_ 698642890
#define NHash2_ -969642118
#define NHash3_ 981908455
#define NI_    0
#define NRelF_ 11
#define NRel_  11
#define NTim_  5
#define NSamp_ 0
#define NCons_ 0
#define NA_    7
#define SizePre_ 5
#define SizeEq_ 4
#define SizeDelay_ 0
#define QNLmax_ 0
#define MAXAux 5
#define NrDymolaTimers_ 0
#define NWhen_ 2
#define NCheckIf_ 0
#define NGlobalHelp_ 67
#define NGlobalHelpI_ 10
#ifndef NExternalObject_
#define NExternalObject_ 1
#endif
#include <moutil.c>
PreNonAliasDef(0)
PreNonAliasDef(1)
PreNonAliasDef(2)
PreNonAliasDef(3)
PreNonAliasDef(4)
PreNonAliasDef(5)
#if !defined(DYM2CCUR)
 DYMOLA_STATIC const char*modelName="clutch_example.simple_clutch_2";
#endif
DYMOLA_STATIC const char*usedLibraries[]={0};
DYMOLA_STATIC const char*dllLibraryPath[]={0};
DYMOLA_STATIC const char*default_dymosim_license_filename=
 "c:/users/patil/appdata/roaming/dynasim/dymola.lic";
#include <dsblock1.c>

/* Define variable names. */

#define Sections_

TranslatedEquations

InitialSection
#if defined(DynSimStruct) || defined(BUILDFMU)
DYNX(W_,0) = 3.141592653589793;
DYNX(W_,8) = 3;
DYNX(W_,9) = 2;
DYNX(W_,10) = 1;
DYNX(W_,11) = 0;
DYNX(W_,12) = -1;
DYNX(W_,14) = 1;
DYNX(W_,15) = 1;
DYNX(W_,16) = false;
DYNX(W_,26) = 219.9114857512855;
DYNX(W_,27) = 62.83185307179586;
DYNX(W_,38) = 1;
#endif
DYNSetAuxString(did_, "NoName", 0);
DYNSetAuxString(did_, "NoName", 1);
#if defined(DynSimStruct) || defined(BUILDFMU)
DYNX(W_,40) = 2;
DYNX(W_,39) = false;
DYNX(W_,41) = 1;
#endif
BoundParameterSection
InitialBoundSection
{
  struct DymStruc0 dummy_DymStruc0;
  dummy_DymStruc0 = (PushModelContext(1,"Modelica.Blocks.Types.ExternalCombiTable1D(\"NoName\", \"NoName\", tab_torque.table, tab_torque.columns, Modelica.Blocks.Types.Smoothness.LinearSegments, 0, 1)")
    Modelica_Blocks_Types_ExternalCombiTable1D_M("NoName", "NoName", 
    RealTemporaryDense( &DYNX(DP_,19), 2, 17, 2), IntegerTemporaryDense( 
    &DYNX(W_,40), 1, 1), 1, 0, 1));
  DYNX(W_,42) = dummy_DymStruc0.id_0member;
PopAllMarks();
}
BoundParameterSection
DYNX(W_,23) = (PushModelContext(1,"Modelica.Math.Vectors.interpolate({mue_pos[1, 1]}, {mue_pos[1, 2]}, 0, 1)")
  Modelica_Math_Vectors_interpolate(RealTemporaryDense( &DYNX(DP_,0), 1, 1), 
  RealTemporaryDense( &DYNX(DP_,1), 1, 1), 0, 1).yi0_0_0member);
PopAllMarks();
InitialSection
#if defined(DynSimStruct) || defined(BUILDFMU)
DYNX(W_,5) = false;
DYNX(W_,6) = false;
DYNX(W_,7) = false;
DYNX(W_,13) = 3;
DYNX(W_,20) = 62.83185307179586;
DYNX(W_,30) = 0;
DYNX(W_,46) = 0;
DYNX(W_,48) = 1;
#endif
InitialSection
InitialStartSection
InitialSection
DefaultSection
InitializeData(0)
InitialSection
DYNX(Aux_,0) = DYNX(W_,48);
DYNX(Aux_,1) = DYNX(W_,13);
DYNX(Aux_,2) = DYNX(W_,6);
DYNX(Aux_,3) = DYNX(W_,5);
InitialSection
InitialSection2
DYNX(W_,43) = 0.0;
  {
    DYNX(W_,43) = 1.0;
  }
DYNX(W_,23) = (PushModelContext(1,"Modelica.Math.Vectors.interpolate({mue_pos[1, 1]}, {mue_pos[1, 2]}, 0, 1)")
  Modelica_Math_Vectors_interpolate(RealTemporaryDense( &DYNX(DP_,0), 1, 1), 
  RealTemporaryDense( &DYNX(DP_,1), 1, 1), 0, 1).yi0_0_0member);
PopAllMarks();
DYNX(W_,48) = DYNX(Aux_,0);
InitialBoundSection
{
  struct DymStruc0 dummy_DymStruc0;
  dummy_DymStruc0 = (PushModelContext(1,"Modelica.Blocks.Types.ExternalCombiTable1D(\"NoName\", \"NoName\", tab_torque.table, tab_torque.columns, Modelica.Blocks.Types.Smoothness.LinearSegments, 0, 1)")
    Modelica_Blocks_Types_ExternalCombiTable1D_M("NoName", "NoName", 
    RealTemporaryDense( &DYNX(DP_,19), 2, 17, 2), IntegerTemporaryDense( 
    &DYNX(W_,40), 1, 1), 1, 0, 1));
  DYNX(W_,42) = dummy_DymStruc0.id_0member;
PopAllMarks();
}
InitialSection
InitialSection2
DYNX(W_,13) = DYNX(Aux_,1);
DYNX(W_,6) = DYNX(Aux_,2);
DYNX(W_,5) = DYNX(Aux_,3);
InitialSection2
Init_=false;InitializeData(2);Init_=true;
EndInitialSection

OutputSection

DynamicsSection

DYNX(Aux_,4) = PRE(DYNX(W_,48), 0);
PRE(DYNX(Aux_,4), 1) = DYNX(Aux_,4);

MixedSystemOfEquations(2, DYNX(DYNhelp,0))
  DYNX(W_,46) = divmacro(DYNX(X_,0),"wr",RealElement( RealTemporaryDense( 
    &DYNX(DP_,54), 1, 12), (SizeType)(DYNX(W_,48))),"i_tm[agear]");
  PopAllMarks();
  DYNX(DYNhelp,2) = RealBmax(DYNX(W_,46), 0);
  DYNX(W_,47) = DYNX(DP_,92)*DYNX(DYNhelp,2);

  MixedModeStartBoolean
  DYNX(Aux_,4) = PRE(DYNX(Aux_,4), 1);
    beginwhenBlock
    whenModelica(GreaterMinor(DYNX(W_,47),"Vx", RealElement( RealTemporaryDense( 
      &DYNX(DP_,68), 1, 12), (SizeType)(PRE(DYNX(W_,48), 0))),"upshift[pre(agear)]",
       0), 0) 
      DYNX(Aux_,4) = PRE(DYNX(W_,48), 0)+1;
    endwhenModelica()

    PopAllMarks();
    whenModelica(LessMinor(DYNX(W_,47),"Vx", RealElement( RealTemporaryDense( 
      &DYNX(DP_,80), 1, 12), (SizeType)(PRE(DYNX(W_,48), 0))),"downshift[pre(agear)]",
       1), 1) 
      DYNX(Aux_,4) = PRE(DYNX(W_,48), 0)-1;
    endwhenModelica()
    endwhenBlock

    PopAllMarks();

  UpdateVariableNamed(DYNX(W_,48),"agear", DYNX(Aux_,4));
  MixedModeInit(2, DYNX(DYNhelp,3))
  ThetaMixedCross(0,0)
  ThetaMixedCross(1,1)
  MixedModeEndBoolean
  UpdateReal(DYNX(W_,46), 1)
  UpdateReal(DYNX(W_,47), 2)
EndMixedSystemOfEquations

DYNX(W_,45) = IF LessTime(1, 0) THEN 0 ELSE IF LessTime(16, 1) THEN 0 ELSE -200;
 /* Linear system of equations to solve. */
DYNX(W_,44) = RememberSimple_(DYNX(W_,44), 0);
SolveScalarLinear(RealElement( RealTemporaryDense( &DYNX(DP_,54), 1, 12), 
  (SizeType)(DYNX(W_,48))),"i_tm[agear]", DYNX(W_,45),"T_ext", DYNX(W_,44),"T_t");
 /* End of Equation Block */ 

DYNX(W_,25) = IF LessTime(1, 0) THEN 0 ELSE IF LessTime(2, 2) THEN 1 ELSE IF 
  LessTime(4, 3) THEN 0 ELSE IF LessTime(16, 1) THEN 1 ELSE 0;
PopAllMarks();
DYNX(W_,24) = DYNX(DP_,4)*DYNX(W_,25);
DYNX(W_,3) = LessEqual(DYNX(W_,24),"fn", 0,"0", 2);
DYNX(W_,29) = IF LessTime(2, 2) THEN 0 ELSE IF LessTime(10, 4) THEN 100 ELSE 0;
DYNX(DYNhelp,5) = RealBmin(DYNX(W_,29)/(double)(100), 1);
DYNX(W_,28) = RealBmax(DYNX(DYNhelp,5), 0.0);
DYNX(DYNhelp,6) = RealBmax(DYNX(X_,2), 62.83185307179586);
DYNX(W_,36) = RealBmin(219.9114857512855, DYNX(DYNhelp,6));
DYNX(W_,32) = (PushModelContext(1,"Modelica.Blocks.Tables.CombiTable1Ds.getTableValue(\ntab_torque.tableID, \n1, \nw_engine_in, \ntab_torque.tableOnFileRead)")
  Modelica_Blocks_Tables_CombiTable1Ds_getTableValue_M(DymStruc0_construct(
  (Integer)(DYNX(W_,42))), 1, DYNX(W_,36), DYNX(W_,43)));
PopModelContext();
DYNX(W_,37) = 0.3183098861837907*DYNX(X_,2)*30;
DYNX(W_,33) = DYNX(DP_,10)*powmacro(DYNX(W_,37),"rpm_engine",3,"3")+DYNX(DP_,11)
  *sqr(DYNX(W_,37))+DYNX(DP_,12)*DYNX(W_,37)+DYNX(DP_,13);
DYNX(DYNhelp,7) = tanh(0.01*(DYNX(W_,36)-219.9114857512855));
DYNX(W_,30) = (DYNX(W_,28)*(DYNX(W_,32)-DYNX(W_,33))+DYNX(W_,33))*(1-
  DYNX(DYNhelp,7));
DYNX(W_,34) = RealBmin(DYNX(W_,30), DYNX(DP_,17));
DYNX(W_,31) = DYNX(W_,34)+DYNX(X_,1);
DYNX(W_,1) = DYNX(W_,23)*DYNX(DP_,3)*DYNX(W_,24);
DYNX(W_,2) = DYNX(DP_,2)*DYNX(W_,1);
DYNX(W_,20) = RealBmin(DYNX(X_,2), 220);
DYNX(W_,18) = DYNX(X_,0)-DYNX(W_,20);

MixedSystemOfEquations(6, DYNX(DYNhelp,8))

   /* Linear system of equations to solve. */
  /* Introducing 3 common subexpressions used in 3 expressions */
  /* Of the common subexpressions 2 are reals, 0 are integers, and 1
     are booleans. */
  DYNX(DYNhelp,14) = IF DYNX(W_,7) THEN 0.0 ELSE IF DYNX(W_,3) THEN 0 ELSE 
    DYNX(DP_,3)*DYNX(W_,24)*(IF DYNX(W_,5) THEN (PushModelContext(1,
    "Modelica.Math.Vectors.interpolate({mue_pos[1, 1]}, {mue_pos[1, 2]}, w_rel, 1)")
    Modelica_Math_Vectors_interpolate(RealTemporaryDense( &DYNX(DP_,0), 1, 1), 
    RealTemporaryDense( &DYNX(DP_,1), 1, 1), DYNX(W_,18), 1).yi0_0_0member)
     ELSE IF DYNX(W_,6) THEN  -(PushModelContext(1,"Modelica.Math.Vectors.interpolate({mue_pos[1, 1]}, {mue_pos[1, 2]}, w_rel, 1)")
    Modelica_Math_Vectors_interpolate(RealTemporaryDense( &DYNX(DP_,0), 1, 1), 
    RealTemporaryDense( &DYNX(DP_,1), 1, 1), DYNX(W_,18), 1).yi0_0_0member)
     ELSE IF PRE(DYNX(W_,13), 2) == 1 THEN (PushModelContext(1,"Modelica.Math.Vectors.interpolate({mue_pos[1, 1]}, {mue_pos[1, 2]}, w_rel, 1)")
    Modelica_Math_Vectors_interpolate(RealTemporaryDense( &DYNX(DP_,0), 1, 1), 
    RealTemporaryDense( &DYNX(DP_,1), 1, 1), DYNX(W_,18), 1).yi0_0_0member)
     ELSE  -(PushModelContext(1,"Modelica.Math.Vectors.interpolate({mue_pos[1, 1]}, {mue_pos[1, 2]},  -w_rel, 1)")
    Modelica_Math_Vectors_interpolate(RealTemporaryDense( &DYNX(DP_,0), 1, 1), 
    RealTemporaryDense( &DYNX(DP_,1), 1, 1),  -DYNX(W_,18), 1).yi0_0_0member));
  PopAllMarks();
  /* Automatic tearing of linear system of 6 simultaneous equations
  gave a linear system of 3 equations for numerical solution.*/
  {
    const char*const varnames_[]={"der(wr)", "sa", "der(w_engine)"};
    const double nominal_[]={1, 1, 1};
    DidLinearSystemOfEquations(J, b, y, 3, DYNX(DYNhelp,15), 42, 
      DYNX(did_->helpvari_vec,0), 10);
    /* Jacobian depending on continuous variables */
    SetNeedFactor(J);
    if (NeedFactor(J)) {
      MatrixZeros(J);
      DYNX(DYNhelp,57) = IF DYNX(W_,7) THEN 1.0 ELSE 0.0;
      DYNX(DYNhelp,58) = DYNX(X_,2) < 220;
      SetMatrixLeading(J, 1, 1, 3, DYNX(DP_,66));
      SetMatrixLeading(J, 1, 2, 3, DYNX(DYNhelp,57));
      SetMatrixLeading(J, 2, 1, 3, 1.0);
      SetMatrixLeading(J, 2, 2, 3,  -(IF DYNX(W_,7) THEN 0.0 ELSE 1.0));
      SetMatrixLeading(J, 2, 3, 3, (-1.0)*(IF DYNX(DYNhelp,58) THEN 1.0 ELSE 0.0));
      SetMatrixLeading(J, 3, 2, 3, (-1.0)*DYNX(DYNhelp,57));
      SetMatrixLeading(J, 3, 3, 3, DYNX(DP_,66));
    }
    SetVector(b, 1, DYNX(W_,44)-DYNX(DYNhelp,14));
    SetVector(b, 2, IF DYNX(W_,7) THEN 0 ELSE IF DYNX(W_,3) THEN 0.0 ELSE IF 
      DYNX(W_,5) THEN  -DYNX(W_,2) ELSE IF DYNX(W_,6) THEN DYNX(W_,2) ELSE IF 
      PRE(DYNX(W_,13), 2) == 1 THEN  -DYNX(W_,2) ELSE DYNX(W_,2));
    SetVector(b, 3, DYNX(W_,31)+DYNX(DYNhelp,14));
    SetVector(y, 1, RememberSimple_(DYNX(F_,0), 1));
    SetVector(y, 2, RememberSimple_(DYNX(W_,4), 2));
    SetVector(y, 3, RememberSimple_(DYNX(F_,2), 3));
    SolveLinearSystemOfEquationsMixed(J, b, y, 1);
    DYNX(F_,0) = GetVector(y, 1);
    DYNX(W_,4) = GetVector(y, 2);
    DYNX(F_,2) = GetVector(y, 3);
    EndStaticLinearSystemOfEquations(J);
  }
  DYNX(W_,21) = IF DYNX(DYNhelp,58) THEN DYNX(F_,2) ELSE 0;
  DYNX(W_,19) = DYNX(F_,0)-DYNX(W_,21);
  DYNX(W_,22) = IF DYNX(W_,7) THEN DYNX(W_,4) ELSE IF DYNX(W_,3) THEN 0 ELSE 
    DYNX(DP_,3)*DYNX(W_,24)*(IF DYNX(W_,5) THEN (PushModelContext(1,
    "Modelica.Math.Vectors.interpolate({mue_pos[1, 1]}, {mue_pos[1, 2]}, w_rel, 1)")
    Modelica_Math_Vectors_interpolate(RealTemporaryDense( &DYNX(DP_,0), 1, 1), 
    RealTemporaryDense( &DYNX(DP_,1), 1, 1), DYNX(W_,18), 1).yi0_0_0member)
     ELSE IF DYNX(W_,6) THEN  -(PushModelContext(1,"Modelica.Math.Vectors.interpolate({mue_pos[1, 1]}, {mue_pos[1, 2]}, w_rel, 1)")
    Modelica_Math_Vectors_interpolate(RealTemporaryDense( &DYNX(DP_,0), 1, 1), 
    RealTemporaryDense( &DYNX(DP_,1), 1, 1), DYNX(W_,18), 1).yi0_0_0member)
     ELSE IF PRE(DYNX(W_,13), 2) == 1 THEN (PushModelContext(1,"Modelica.Math.Vectors.interpolate({mue_pos[1, 1]}, {mue_pos[1, 2]}, w_rel, 1)")
    Modelica_Math_Vectors_interpolate(RealTemporaryDense( &DYNX(DP_,0), 1, 1), 
    RealTemporaryDense( &DYNX(DP_,1), 1, 1), DYNX(W_,18), 1).yi0_0_0member)
     ELSE  -(PushModelContext(1,"Modelica.Math.Vectors.interpolate({mue_pos[1, 1]}, {mue_pos[1, 2]},  -w_rel, 1)")
    Modelica_Math_Vectors_interpolate(RealTemporaryDense( &DYNX(DP_,0), 1, 1), 
    RealTemporaryDense( &DYNX(DP_,1), 1, 1),  -DYNX(W_,18), 1).yi0_0_0member));
  PopAllMarks();
   /* End of Equation Block */ 


  MixedModeStartBoolean
  UpdateVariableNamed(DYNX(W_,5),"startForward", PRE(DYNX(W_,13), 2) == 0 AND (
    Greater(DYNX(W_,4),"sa", DYNX(W_,2),"tau0_max", 3) OR PRE(DYNX(W_,5), 3)
     AND Greater(DYNX(W_,4),"sa", DYNX(W_,1),"tau0", 4)) OR PRE(DYNX(W_,13), 2)
     == -1 AND Greater(DYNX(W_,18),"w_rel", DYNX(DP_,5),"w_small", 5) OR initial
    () AND Greater(DYNX(W_,18),"w_rel", 0,"0", 6));
  UpdateVariableNamed(DYNX(W_,6),"startBackward", PRE(DYNX(W_,13), 2) == 0 AND (
    Less(DYNX(W_,4),"sa",  -DYNX(W_,2)," -tau0_max", 7) OR PRE(DYNX(W_,6), 4)
     AND Less(DYNX(W_,4),"sa",  -DYNX(W_,1)," -tau0", 8)) OR PRE(DYNX(W_,13), 2)
     == 1 AND Less(DYNX(W_,18),"w_rel",  -DYNX(DP_,5)," -w_small", 9) OR initial
    () AND Less(DYNX(W_,18),"w_rel", 0,"0", 10));
  UpdateVariableNamed(DYNX(W_,7),"locked",  NOT DYNX(W_,3) AND  NOT (PRE(
    DYNX(W_,13), 2) == 1 OR DYNX(W_,5) OR PRE(DYNX(W_,13), 2) == -1 OR 
    DYNX(W_,6)));
  MixedModeInit(8, DYNX(DYNhelp,59))
  ThetaMixedCross(0,3)
  ThetaMixedCross(1,4)
  ThetaMixedCross(2,5)
  ThetaMixedCross(3,6)
  ThetaMixedCross(4,7)
  ThetaMixedCross(5,8)
  ThetaMixedCross(6,9)
  ThetaMixedCross(7,10)
  MixedModeEndBoolean
  UpdateReal(DYNX(F_,2), 1)
  UpdateReal(DYNX(W_,21), 2)
  UpdateReal(DYNX(W_,19), 3)
  UpdateReal(DYNX(W_,4), 4)
  UpdateReal(DYNX(W_,22), 5)
  UpdateReal(DYNX(F_,0), 6)
EndMixedSystemOfEquations

DYNX(W_,35) = DYNX(W_,30)-DYNX(W_,34);
DYNX(F_,1) = DYNX(DP_,18)*(DYNX(W_,35)-DYNX(X_,1));

AcceptedSection1
DYNX(W_,13) = IF DYNX(W_,3) THEN 2 ELSE IF (PRE(DYNX(W_,13), 2) == 1 OR PRE(
  DYNX(W_,13), 2) == 2 OR DYNX(W_,5)) AND Greater(DYNX(W_,18),"w_rel", 0,"0", 6)
   THEN 1 ELSE IF (PRE(DYNX(W_,13), 2) == -1 OR PRE(DYNX(W_,13), 2) == 2 OR 
  DYNX(W_,6)) AND Less(DYNX(W_,18),"w_rel", 0,"0", 10) THEN -1 ELSE 0;

AcceptedSection2
DYNX(W_,17) = DYNX(W_,22)*DYNX(W_,18);

DefaultSection
InitializeData(1)
EndTranslatedEquations

#include <dsblock6.c>

PreNonAliasNew(0)
StartNonAlias(0)
DeclareVariable("pi", "", 3.141592653589793, 0.0,0.0,0.0,0,513)
DeclareParameter("mue_pos[1, 1]", "[w,mue] positive sliding friction coefficient (w_rel>=0)",\
 0, 0.0, 0.0,0.0,0.0,0,560)
DeclareParameter("mue_pos[1, 2]", "[w,mue] positive sliding friction coefficient (w_rel>=0)",\
 1, 0.5, 0.0,0.0,0.0,0,560)
DeclareParameter("peak", "peak*mue_pos[1,2] = maximum value of mue for w_rel==0 [1]",\
 2, 1, 1.0,1E+100,0.0,0,560)
DeclareParameter("cgeo", "Geometry constant containing friction distribution assumption",\
 3, 1, 0.0,1E+100,0.0,0,560)
DeclareParameter("fn_max", "Maximum normal force [N]", 4, 1500, 0.0,1E+100,0.0,0,560)
DeclareParameter("w_small", "Relative angular velocity near to zero if jumps due to a reinit(..) of the velocity can occur (set to low value only if such impulses can occur) [rad/s]",\
 5, 10000000000.0, 0.0,0.0,0.0,0,560)
DeclareAlias2("w_relfric", "Relative angular velocity between frictional surfaces [rad/s]",\
 "w_rel", 1, 5, 18, 0)
DeclareAlias2("a_relfric", "Relative angular acceleration between frictional surfaces [rad/s2]",\
 "der(w_rel)", 1, 5, 19, 0)
DeclareVariable("tau0", "Friction torque for w_relfric=0 and forward sliding [N.m]",\
 0.0, 0.0,0.0,0.0,0,640)
DeclareVariable("tau0_max", "Maximum friction torque for w_relfric=0 and locked [N.m]",\
 0.0, 0.0,0.0,0.0,0,640)
DeclareVariable("free", "= true, if frictional element is not active [:#(type=Boolean)]",\
 false, 0.0,0.0,0.0,0,642)
DeclareVariable("sa", "Path parameter of friction characteristic tau = f(a_relfric) [1]",\
 0.0, 0.0,0.0,0.0,0,512)
DeclareVariable("startForward", "= true, if w_relfric=0 and start of forward sliding [:#(type=Boolean)]",\
 false, 0.0,0.0,0.0,0,722)
DeclareVariable("startBackward", "= true, if w_relfric=0 and start of backward sliding [:#(type=Boolean)]",\
 false, 0.0,0.0,0.0,0,722)
DeclareVariable("locked", "true, if w_rel=0 and not sliding [:#(type=Boolean)]",\
 false, 0.0,0.0,0.0,0,706)
DeclareVariable("Unknown", "Value of mode is not known [:#(type=Integer)]", 3, \
0.0,0.0,0.0,0,517)
DeclareVariable("Free", "Element is not active [:#(type=Integer)]", 2, 0.0,0.0,\
0.0,0,517)
DeclareVariable("Forward", "w_relfric > 0 (forward sliding) [:#(type=Integer)]",\
 1, 0.0,0.0,0.0,0,517)
DeclareVariable("Stuck", "w_relfric = 0 (forward sliding, locked or backward sliding) [:#(type=Integer)]",\
 0, 0.0,0.0,0.0,0,517)
DeclareVariable("Backward", "w_relfric < 0 (backward sliding) [:#(type=Integer)]",\
 -1, 0.0,0.0,0.0,0,517)
DeclareVariable("mode", "Mode of friction (-1: backward sliding, 0: stuck, 1: forward sliding, 2: inactive, 3: unknown) [:#(type=Integer)]",\
 3, -1.0,3.0,0.0,0,660)
DeclareVariable("unitAngularAcceleration", "[rad/s2]", 1, 0.0,0.0,0.0,0,1537)
DeclareVariable("unitTorque", "[N.m]", 1, 0.0,0.0,0.0,0,1537)
DeclareVariable("useHeatPort", "=true, if heatPort is enabled [:#(type=Boolean)]",\
 false, 0.0,0.0,0.0,0,1539)
DeclareVariable("lossPower", "Loss power leaving component via heatPort (> 0, if heat is flowing out of component) [W]",\
 0.0, 0.0,0.0,0.0,0,512)
DeclareVariable("w_rel", "[rad/s]", 0.0, 0.0,0.0,0.0,0,512)
DeclareVariable("der(w_rel)", "[rad/s2]", 0.0, 0.0,0.0,0.0,0,512)
DeclareVariable("wl", "[rad/s]", 62.83185307179586, 0.0,0.0,0.0,0,512)
DeclareVariable("der(wl)", "[rad/s2]", 0.0, 0.0,0.0,0.0,0,512)
DeclareState("wr", "[rad/s]", 0, 0, 0.0,0.0,0.0,0,560)
DeclareDerivative("der(wr)", "[rad/s2]", 0.0, 0.0,0.0,0.0,0,512)
DeclareVariable("tau", "[J]", 0.0, 0.0,0.0,0.0,0,512)
DeclareVariable("mue0", "Friction coefficient for w=0 and forward sliding", 0.0,\
 0.0,0.0,0.0,0,513)
DeclareVariable("fn", "Normal force (fn=fn_max*f_normalized) [N]", 0.0, 0.0,0.0,\
0.0,0,640)
DeclareVariable("f_normalized", "[1]", 0.0, 0.0,0.0,0.0,0,640)
DeclareParameter("k3max", "", 6, -1.246E-006, 0.0,0.0,0.0,0,560)
DeclareParameter("k2max", "", 7, 0.0001471, 0.0,0.0,0.0,0,560)
DeclareParameter("k1max", "", 8, 6.511, 0.0,0.0,0.0,0,560)
DeclareParameter("k0max", "", 9, -2228.7, 0.0,0.0,0.0,0,560)
DeclareParameter("k3min", "[J.s3.rad-3]", 10, -1.59E-008, 0.0,0.0,0.0,0,560)
DeclareParameter("k2min", "[J.s2.rad-2]", 11, 8.752E-005, 0.0,0.0,0.0,0,560)
DeclareParameter("k1min", "[J.s.rad-1]", 12, -0.197569, 0.0,0.0,0.0,0,560)
DeclareParameter("k0min", "[N.m]", 13, -4.22681, 0.0,0.0,0.0,0,560)
DeclareVariable("w_eng_max", "maximum engine rotation [rad/s]", 219.9114857512855,\
 0.0,0.0,0.0,0,513)
DeclareVariable("w_eng_idl", " engine idle speed [rad/s]", 62.83185307179586, \
0.0,0.0,0.0,0,513)
DeclareParameter("axh", "Upper acceleration limit [m/s2]", 14, 0.9, 0.0,0.0,0.0,\
0,560)
DeclareParameter("axl", "Lower acceleration limit [m/s2]", 15, 0.8, 0.0,0.0,0.0,\
0,560)
DeclareParameter("E_factor", "", 16, 0.9, 0.0,0.0,0.0,0,560)
DeclareParameter("Tsplit", "[N.m]", 17, 700, 0.0,0.0,0.0,0,560)
DeclareParameter("k", "Boost pressure coefficient [s-1]", 18, 1, 0.0,0.0,0.0,0,560)
DeclareVariable("perc_throttle", "[1]", 0.0, 0.0,0.0,0.0,0,640)
DeclareVariable("Aped", "Accelerator pedal position [%] []", 0.0, 0.0,0.0,0.0,0,640)
DeclareVariable("torque", "engine torque [N.m]", 0, 0.0,0.0,0.0,0,512)
DeclareVariable("Te", "[N.m]", 0.0, 0.0,0.0,0.0,0,512)
DeclareVariable("max_torque", "[N.m]", 0.0, 0.0,0.0,0.0,0,512)
DeclareVariable("min_torque", "[N.m]", 0.0, 0.0,0.0,0.0,0,512)
DeclareVariable("Tbase", "[N.m]", 0.0, 0.0,0.0,0.0,0,512)
DeclareVariable("Tdynreq", "[N.m]", 0.0, 0.0,0.0,0.0,0,512)
DeclareState("Ttop", "[N.m]", 1, 0.0, 0.0,0.0,0.0,0,560)
DeclareDerivative("der(Ttop)", "[W]", 0.0, 0.0,0.0,0.0,0,512)
DeclareAlias2("T_e", "[N.m]", "Te", 1, 5, 31, 0)
DeclareVariable("w_engine_in", "[rad/s]", 0.0, 0.0,0.0,0.0,0,512)
DeclareState("w_engine", "engine speed in rad/s [rad/s]", 2, 62.83185307179586, \
0.0,0.0,0.0,0,560)
DeclareDerivative("der(w_engine)", "der(engine speed in rad/s) [rad/s2]", 0.0, \
0.0,0.0,0.0,0,512)
DeclareVariable("rpm_engine", "engine speed in rpm [rad/s]", 0.0, 0.0,0.0,0.0,0,512)
DeclareVariable("tab_torque.nout", "Number of outputs [:#(type=Integer)]", 1, \
0.0,0.0,0.0,0,517)
DeclareAlias2("tab_torque.u", "Connector of Real input signal [rad/s]", \
"w_engine_in", 1, 5, 36, 0)
DeclareAlias2("tab_torque.y[1]", "Connector of Real output signals", \
"max_torque", 1, 5, 32, 0)
DeclareVariable("tab_torque.tableOnFile", "= true, if table is defined on file or in function usertab [:#(type=Boolean)]",\
 false, 0.0,0.0,0.0,0,515)
DeclareParameter("tab_torque.table[1, 1]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 19, 0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[1, 2]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 20, 0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[2, 1]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 21, 62.8318, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[2, 2]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 22, 1660.0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[3, 1]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 23, 73.3038, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[3, 2]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 24, 1880.0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[4, 1]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 25, 83.775, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[4, 2]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 26, 2240.0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[5, 1]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 27, 94.247, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[5, 2]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 28, 2900.0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[6, 1]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 29, 104.719, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[6, 2]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 30, 3550.0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[7, 1]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 31, 115.191, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[7, 2]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 32, 3550.0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[8, 1]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 33, 125.663, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[8, 2]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 34, 3550.0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[9, 1]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 35, 136.135, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[9, 2]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 36, 3550.0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[10, 1]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 37, 146.607, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[10, 2]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 38, 3550.0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[11, 1]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 39, 157.079, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[11, 2]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 40, 3470.0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[12, 1]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 41, 167.551, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[12, 2]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 42, 3310.0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[13, 1]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 43, 178.023, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[13, 2]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 44, 3120.0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[14, 1]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 45, 188.495, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[14, 2]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 46, 2880.0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[15, 1]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 47, 198.967, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[15, 2]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 48, 2660.0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[16, 1]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 49, 209.439, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[16, 2]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 50, 1680.0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[17, 1]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 51, 219.911, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.table[17, 2]", "Table matrix (grid = first column; e.g., table=[0,2])",\
 52, 0.0, 0.0,0.0,0.0,0,560)
DeclareParameter("tab_torque.verboseRead", "= true, if info message that file is loading is to be printed [:#(type=Boolean)]",\
 53, true, 0.0,0.0,0.0,0,562)
DeclareVariable("tab_torque.columns[1]", "Columns of table to be interpolated [:#(type=Integer)]",\
 2, 0.0,0.0,0.0,0,517)
DeclareVariable("tab_torque.smoothness", "Smoothness of table interpolation [:#(type=Modelica.Blocks.Types.Smoothness)]",\
 1, 1.0,5.0,0.0,0,517)
DeclareVariable("tab_torque.tableID.id", "[:#(type=Integer)]", 0, 0.0,0.0,0.0,0,2565)
DeclareVariable("tab_torque.tableOnFileRead", "= 1, if table was successfully read from file",\
 0.0, 0.0,0.0,0.0,0,2561)
DeclareAlias2("T", "[N.m]", "Te", 1, 5, 31, 0)
DeclareVariable("T_t", "[J]", 0.0, 0.0,0.0,0.0,0,640)
DeclareVariable("T_ext", "", 0.0, 0.0,0.0,0.0,0,640)
DeclareVariable("ww", "", 0, 0.0,0.0,0.0,0,512)
DeclareAlias2("wt", "[rad/s]", "wr", 1, 1, 0, 0)
DeclareVariable("Vx", "", 0.0, 0.0,0.0,0.0,0,512)
DeclareVariable("agear", "[:#(type=Integer)]", 1, 0.0,0.0,0.0,0,660)
DeclareParameter("i_tm[1]", "", 54, 14.356, 0.0,0.0,0.0,0,560)
DeclareParameter("i_tm[2]", "", 55, 11.7239, 0.0,0.0,0.0,0,560)
DeclareParameter("i_tm[3]", "", 56, 9.0363, 0.0,0.0,0.0,0,560)
DeclareParameter("i_tm[4]", "", 57, 7.0941, 0.0,0.0,0.0,0,560)
DeclareParameter("i_tm[5]", "", 58, 5.5382, 0.0,0.0,0.0,0,560)
DeclareParameter("i_tm[6]", "", 59, 4.3478, 0.0,0.0,0.0,0,560)
DeclareParameter("i_tm[7]", "", 60, 3.4363, 0.0,0.0,0.0,0,560)
DeclareParameter("i_tm[8]", "", 61, 2.6977, 0.0,0.0,0.0,0,560)
DeclareParameter("i_tm[9]", "", 62, 2.0783, 0.0,0.0,0.0,0,560)
DeclareParameter("i_tm[10]", "", 63, 1.6316, 0.0,0.0,0.0,0,560)
DeclareParameter("i_tm[11]", "", 64, 1.2738, 0.0,0.0,0.0,0,560)
DeclareParameter("i_tm[12]", "", 65, 1.0, 0.0,0.0,0.0,0,560)
DeclareParameter("Je", "[J.s2.rad-1]", 66, 1, 0.0,0.0,0.0,0,560)
DeclareParameter("Jw", "", 67, 1, 0.0,0.0,0.0,0,560)
DeclareParameter("upshift[1]", "", 68, 2.31, 0.0,0.0,0.0,0,560)
DeclareParameter("upshift[2]", "", 69, 3.0, 0.0,0.0,0.0,0,560)
DeclareParameter("upshift[3]", "", 70, 3.81, 0.0,0.0,0.0,0,560)
DeclareParameter("upshift[4]", "", 71, 4.86, 0.0,0.0,0.0,0,560)
DeclareParameter("upshift[5]", "", 72, 6.25, 0.0,0.0,0.0,0,560)
DeclareParameter("upshift[6]", "", 73, 8.0, 0.0,0.0,0.0,0,560)
DeclareParameter("upshift[7]", "", 74, 10.02, 0.0,0.0,0.0,0,560)
DeclareParameter("upshift[8]", "", 75, 12.8, 0.0,0.0,0.0,0,560)
DeclareParameter("upshift[9]", "", 76, 16.65, 0.0,0.0,0.0,0,560)
DeclareParameter("upshift[10]", "", 77, 21.2, 0.0,0.0,0.0,0,560)
DeclareParameter("upshift[11]", "", 78, 27.1, 0.0,0.0,0.0,0,560)
DeclareParameter("upshift[12]", "", 79, 1000000.0, 0.0,0.0,0.0,0,560)
DeclareParameter("downshift[1]", "", 80, -1000000.0, 0.0,0.0,0.0,0,560)
DeclareParameter("downshift[2]", "", 81, 2.2, 0.0,0.0,0.0,0,560)
DeclareParameter("downshift[3]", "", 82, 2.28, 0.0,0.0,0.0,0,560)
DeclareParameter("downshift[4]", "", 83, 3.5, 0.0,0.0,0.0,0,560)
DeclareParameter("downshift[5]", "", 84, 4.5, 0.0,0.0,0.0,0,560)
DeclareParameter("downshift[6]", "", 85, 6.0, 0.0,0.0,0.0,0,560)
DeclareParameter("downshift[7]", "", 86, 7.5, 0.0,0.0,0.0,0,560)
DeclareParameter("downshift[8]", "", 87, 9.4, 0.0,0.0,0.0,0,560)
DeclareParameter("downshift[9]", "", 88, 12.0, 0.0,0.0,0.0,0,560)
DeclareParameter("downshift[10]", "", 89, 15.5, 0.0,0.0,0.0,0,560)
DeclareParameter("downshift[11]", "", 90, 20.0, 0.0,0.0,0.0,0,560)
DeclareParameter("downshift[12]", "", 91, 25.0, 0.0,0.0,0.0,0,560)
DeclareParameter("rw", "", 92, 0.5, 0.0,0.0,0.0,0,560)
EndNonAlias(0)

#define DymolaHaveUpdateInitVars 1
#include <dsblock5.c>

DYMOLA_STATIC void UpdateInitVars(double*time, double* X_, double* XD_, double* U_, double* DP_, int IP_[], Dymola_bool LP_[], double* F_, double* Y_, double* W_, double QZ_[], double duser_[], int iuser_[], void*cuser_[],struct DYNInstanceData*did_) {
static Real initStore[1];
}
StartDataBlock
StartPreBlock
pre(DYNX(W_,48),"agear", 1, 0);
pre(DYNX(Aux_,4),"agear_alg1.0.", 1, 1);
pre(DYNX(W_,13),"mode", 3, 2);
pre(DYNX(W_,6),"startBackward", false, 4);
pre(DYNX(W_,5),"startForward", false, 3);
EndPreBlock
StartEqBlock
DoRemember_(DYNX(W_,44), 0.0, 0);
DoRemember_(DYNX(F_,2), 0.0, 3);
DoRemember_(DYNX(F_,0), 0.0, 1);
DoRemember_(DYNX(W_,4), 0.0, 2);
EndEqBlock
UpdateQEvaluate(2)
EndDataBlock
