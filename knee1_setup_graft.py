#############################################################
#For a full knee model with hole in femoral condyle
#############################################################


from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *
from caeModules import *

# stop jnl file from using getSequenceFromMask
session.journalOptions.setValues(replayGeometry=COORDINATE, recoverGeometry=COORDINATE)

dispQuant=9

mdb.ModelFromInputFile(inputFileName='', name='Knee')
Mod = mdb.models['Knee']

del mdb.models['Model-1']

# delete interactions made to avoid ties
del mdb.models['Knee'].interactionProperties['CT_FCART_WITH_CYL_Y_10']
del mdb.models['Knee'].interactionProperties['CT_FEMUR_WITH_CYL_Y_10']

mdb.models['Knee'].interactions.delete(('CT_FCART_WITH_CYL_Y_10-1', 
    'CT_FEMUR_WITH_CYL_Y_10-1', ))
# interactions
Mod.ContactProperty('IntProp-1')
Mod.interactionProperties['IntProp-1'].NormalBehavior(
	allowSeparation=ON, constraintEnforcementMethod=DEFAULT, 
	pressureOverclosure=HARD)
Mod.interactionProperties['IntProp-1'].TangentialBehavior(
	dependencies=0, directionality=ISOTROPIC, elasticSlipStiffness=None, 
	formulation=PENALTY, fraction=0.005, maximumElasticSlip=FRACTION, 
	pressureDependency=OFF, shearStressLimit=None, slipRateDependency=OFF, 
	table=((0.1, ), ), temperatureDependency=OFF)		
	
Mod.ContactProperty('IntProp-2')
Mod.interactionProperties['IntProp-2'].NormalBehavior(
	allowSeparation=ON, constraintEnforcementMethod=DEFAULT, 
	pressureOverclosure=HARD)
Mod.interactionProperties['IntProp-2'].TangentialBehavior(
	dependencies=0, directionality=ISOTROPIC, elasticSlipStiffness=None, 
	formulation=PENALTY, fraction=0.005, maximumElasticSlip=FRACTION, 
	pressureDependency=OFF, shearStressLimit=None, slipRateDependency=OFF, 
	table=((0.15, ), ), temperatureDependency=OFF)			
	
# surface interactions

	
Mod.SurfaceToSurfaceContactStd(adjustMethod=OVERCLOSED, 
	clearanceRegion=None, createStepName='Initial', datumAxis=None, 
	initialClearance=OMIT, interactionProperty='IntProp-1', master=
	Mod.rootAssembly.surfaces['SF_FCART_SELF_CONTACT'], name='Int-5', 
	slave=Mod.rootAssembly.surfaces['SF_TCART_MED_SELF_CONTACT'], 
	sliding=FINITE, thickness=ON)	

Mod.SurfaceToSurfaceContactStd(adjustMethod=OVERCLOSED, 
	clearanceRegion=None, createStepName='Initial', datumAxis=None, 
	initialClearance=OMIT, interactionProperty='IntProp-1', master=
	Mod.rootAssembly.surfaces['SF_FCART_SELF_CONTACT'], name='Int-6', 
	slave=Mod.rootAssembly.surfaces['SF_TCART_LAT_SELF_CONTACT'], 
	sliding=FINITE, thickness=ON)	
	
# reference points	
RPfemur = Mod.rootAssembly.ReferencePoint(point=(59,80,143)).id 
RPtibia = Mod.rootAssembly.ReferencePoint(point=(57,83,68)).id		

# create steps after initial
Mod.StaticStep(initialInc=0.1, matrixSolver=DIRECT,
    matrixStorage=UNSYMMETRIC, name='displacement', nlgeom=ON, previous=
    'Initial')

Mod.StaticStep(initialInc=0.1, name='relax', previous=
    'displacement')
Mod.StaticStep(name='Move', previous='relax', matrixSolver=DIRECT, matrixStorage=UNSYMMETRIC)
Mod.StaticStep(name='Load1', previous='Move', matrixSolver=DIRECT, matrixStorage=UNSYMMETRIC)
Mod.steps['Move'].setValues(nlgeom=ON)	


# couplings
mdb.models['Knee'].Coupling(controlPoint=Region(referencePoints=(
    mdb.models['Knee'].rootAssembly.referencePoints[RPfemur], )), couplingType=
    KINEMATIC, influenceRadius=WHOLE_SURFACE, localCsys=None, name='Constraint-1', surface=
    mdb.models['Knee'].rootAssembly.sets['NS_FEMUR_WITH_ZMAX'], u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON)	
	
mdb.models['Knee'].Coupling(controlPoint=Region(referencePoints=(
    mdb.models['Knee'].rootAssembly.referencePoints[RPtibia], )), couplingType=
    KINEMATIC, influenceRadius=WHOLE_SURFACE, localCsys=None, name='Constraint-2', surface=
    mdb.models['Knee'].rootAssembly.sets['NS_TIBIA_WITH_ZMIN'], u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON)		

# BCs

Mod.DisplacementBC(amplitude=UNSET, createStepName='Initial', 
    distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=None, name=
    'Fix_tib', region=Region(referencePoints=(Mod.rootAssembly.referencePoints[RPtibia], )), u1=0.0, u2=
    0.0, u3=0.0, ur1=0.0, ur2=0.0, ur3=0.0)
	
Mod.DisplacementBC(amplitude=UNSET, createStepName='Move', 
    distributionType=UNIFORM, fieldName='', fixed=OFF, localCsys=None, name=
    'Displace_fem', region=Region(referencePoints=(Mod.rootAssembly.referencePoints[RPfemur], )), u1=0.0, u2=
    0.0, u3=-1, ur1=0.0, ur2=0.0, ur3=0.0)
Mod.boundaryConditions['Displace_fem'].setValuesInStep(stepName=
    'Load1', u3=FREED)		
	
Mod.ConcentratedForce(cf3=-500.0, createStepName='Load1', 
    distributionType=UNIFORM, field='', localCsys=None, name='Load-1', region=
    Region(referencePoints=(Mod.rootAssembly.referencePoints[RPfemur], )))
# Mod.loads['Load-1'].setValuesInStep(cf3=-1000.0, stepName='Load2')

Mod.BoundaryCondition(name='Displace_fem-rot', objectToCopy=
    Mod.boundaryConditions['Displace_fem'], toStepName='Move')
Mod.boundaryConditions['Displace_fem-rot'].setValues(ur1=0.1)
Mod.boundaryConditions['Displace_fem-rot'].suppress()

Mod.BoundaryCondition(name='Fix_tib-free', objectToCopy=
    Mod.boundaryConditions['Fix_tib'], toStepName='Initial')
Mod.boundaryConditions['Fix_tib-free'].setValuesInStep(stepName=
    'Move', u2=FREED, ur2=FREED, ur3=FREED)
Mod.boundaryConditions['Fix_tib-free'].deactivate('Load1')
Mod.boundaryConditions['Fix_tib-free'].suppress()

mdb.models['Knee'].VelocityBC(amplitude=UNSET, createStepName='Load1', 
    distributionType=UNIFORM, fieldName='', localCsys=None, name='refix-tib', 
    region=Region(referencePoints=(
    mdb.models['Knee'].rootAssembly.referencePoints[RPtibia], )), v1=0.0, v2=0.0, 
    v3=0.0, vr1=0.0, vr2=0.0, vr3=0.0)
Mod.boundaryConditions['refix-tib'].suppress()	
mdb.models['Knee'].boundaryConditions['Fix_tib'].setValuesInStep(
    stepName='displacement', u3=-1.0)
mdb.models['Knee'].boundaryConditions['Fix_tib'].setValuesInStep(
    stepName='Move', u3=0.0)

# outputs
mdb.models['Knee'].historyOutputRequests['H-Output-1'].setValuesInStep(stepName=
    'Load1', variables=('CAREA', 'ALLAE', 'ALLCD', 'ALLDMD', 'ALLEE', 'ALLFD', 
    'ALLIE', 'ALLJD', 'ALLKE', 'ALLKL', 'ALLPD', 'ALLQB', 'ALLSE', 'ALLSD', 
    'ALLVD', 'ALLWK', 'ETOTAL'))
mdb.models['Knee'].fieldOutputRequests['F-Output-1'].setValuesInStep(stepName=
    'Load1', variables=('S', 'PE', 'PEEQ', 'PEMAG', 'LE', 'U', 'RF', 'CF', 
    'CSTRESS', 'CDISP', 'CFORCE', 'CNAREA'))
	
# rotation	
mdb.models['Knee'].rootAssembly.rotate(angle=-25.0, axisDirection=(0.0, 0.0, 1.0), axisPoint=(57.0, 83.0, 68.0), instanceList=('PART-1-1', ))	



#----------------------------------------------------------------------------------------
# Plug Stuff 
#----------------------------------------------------------------------------------------



Mod.parts['PART-1'].deleteElement(elements=[Mod.parts['PART-1'].sets['PT_CYL_Y_10']])



Mod.rootAssembly.suppressFeatures(featureNames=(
    'plug6-1', ))
mdb.ModelFromInputFile(inputFileName=
    '', name=
    'Knee4Plug6')
mdb.models['Knee4Plug6'].parts.changeKey(fromName='PART-1', toName='plug6')
Mod.Part('plug6', mdb.models['Knee4Plug6'].parts['plug6'])
Mod.copyMaterials(sourceModel=mdb.models['Knee4Plug6'])
Mod.copySections(sourceModel=mdb.models['Knee4Plug6'])
Mod.rootAssembly.Instance(dependent=ON, name='plug6-1', part=
    Mod.parts['plug6'])
del mdb.models['Knee4Plug6']
Mod.rootAssembly.regenerate()


setsa = []
for i in range(0,255):
    setsa.append(mdb.models['Knee'].parts['PART-1'].sets['PTGS'+str(i)+'_TIBIA'])

    
leaf = dgm.LeafFromSets(sets=setsa)
dg = session.DisplayGroup(leaf=leaf, name='DisplayGroup-2')



Mod.parts['PART-1'].DatumPlaneByThreePoints(point1=
    Mod.parts['PART-1'].nodes[228700], point2=
    Mod.parts['PART-1'].nodes[230013], point3=
    Mod.parts['PART-1'].nodes[230009])
Mod.parts['PART-1'].DatumPointByMidPoint(point1=
    Mod.parts['PART-1'].nodes[229526], point2=
    Mod.parts['PART-1'].nodes[229043])
Mod.parts['PART-1'].DatumPlaneByOffset(flip=SIDE2, offset=-dispQuant,
    plane=Mod.parts['PART-1'].datums[516])
Mod.parts['PART-1'].DatumPointByProjOnFace(face=
    Mod.parts['PART-1'].datums[518], point=
    Mod.parts['PART-1'].datums[517])


#Set the coodinate system and the centre position of the plug
Mod.parts['plug6'].DatumPointByMidPoint(point1=
    Mod.parts['plug6'].nodes[675], point2=
    Mod.parts['plug6'].nodes[605])
Mod.parts['plug6'].DatumCsysByThreePoints(coordSysType=
    CARTESIAN, name='plug_CSYS', origin=
    Mod.parts['plug6'].nodes[1806], point1=
    Mod.parts['plug6'].nodes[918], point2=
    Mod.parts['plug6'].nodes[1579])

Mod.rootAssembly.regenerate()


a1 = mdb.models['Knee'].rootAssembly
q1 = a1.instances['plug6-1'].elemFaces
d1 = a1.instances['PART-1-1'].datums
a1.FaceToFace(movablePlane=q1[54576], fixedPlane=d1[518], flip=OFF, 
    clearance=0.0)
a1 = mdb.models['Knee'].rootAssembly
a1.translate(instanceList=('plug6-1', ), vector=(-6.222186, -1.290669, 
    -0.423916))
#a1.translate(instanceList=('plug6-1', ), vector=(0.004648, -3.854734, 
#    -0.549638))
#----------------------------------------------------------------------------------------
# 
#----------------------------------------------------------------------------------------



#----------------------------------------------------------------------------------------
# Surface Descriptions 
#----------------------------------------------------------------------------------------


a = mdb.models['Knee'].rootAssembly
f1 = a.instances['plug6-1'].elements
face1Elements1 = f1[35:36]+f1[62:63]+f1[84:85]+f1[140:141]+f1[148:149]+\
    f1[159:160]+f1[176:177]+f1[216:217]+f1[262:263]+f1[264:265]+f1[267:268]+\
    f1[350:352]+f1[360:361]+f1[458:459]+f1[461:462]+f1[470:471]+f1[474:475]+\
    f1[512:513]+f1[579:580]+f1[582:583]+f1[688:689]+f1[696:697]+f1[720:721]+\
    f1[737:739]+f1[746:747]+f1[773:774]+f1[917:918]+f1[923:924]+f1[925:926]+\
    f1[929:930]+f1[994:995]+f1[1032:1033]+f1[1034:1035]+f1[1043:1044]+\
    f1[1045:1046]+f1[1053:1054]+f1[1065:1066]+f1[1102:1103]+f1[1106:1107]+\
    f1[1111:1112]+f1[1121:1122]+f1[1147:1148]+f1[1180:1181]+f1[1261:1262]+\
    f1[1263:1264]+f1[1280:1281]+f1[1314:1315]+f1[1373:1374]+f1[1422:1423]+\
    f1[1496:1497]+f1[1521:1522]+f1[1571:1572]+f1[1575:1576]+f1[1606:1607]+\
    f1[1612:1613]+f1[1619:1620]+f1[1661:1662]+f1[1688:1689]+f1[1777:1778]+\
    f1[1849:1850]+f1[1915:1916]+f1[1940:1941]+f1[1948:1949]+f1[1966:1967]+\
    f1[2033:2034]+f1[2043:2044]+f1[2047:2048]+f1[2111:2112]+f1[2125:2126]+\
    f1[2128:2129]+f1[2158:2159]+f1[2162:2163]+f1[2169:2170]+f1[2171:2172]+\
    f1[2173:2174]+f1[2204:2205]+f1[2243:2244]+f1[2282:2283]+f1[2288:2289]+\
    f1[2290:2291]+f1[2292:2293]+f1[2351:2352]+f1[2375:2376]+f1[2396:2397]+\
    f1[2490:2491]+f1[2511:2512]+f1[2524:2525]+f1[2556:2557]+f1[2559:2561]+\
    f1[2593:2594]+f1[2604:2605]+f1[2614:2615]+f1[2629:2630]+f1[2650:2651]+\
    f1[2700:2701]+f1[2733:2734]+f1[2737:2738]+f1[2745:2746]+f1[2755:2756]+\
    f1[2892:2893]+f1[2970:2971]+f1[2981:2982]+f1[3018:3019]+f1[3179:3180]+\
    f1[3241:3242]+f1[3307:3308]+f1[3329:3330]+f1[3337:3338]+f1[3365:3366]+\
    f1[3382:3384]+f1[3418:3420]+f1[3449:3450]+f1[3471:3472]+f1[3476:3477]+\
    f1[3480:3481]+f1[3508:3509]+f1[3554:3555]+f1[3560:3561]+f1[3592:3593]+\
    f1[3661:3662]+f1[3744:3745]+f1[3795:3796]+f1[3902:3903]+f1[3938:3939]+\
    f1[3973:3974]+f1[4088:4089]+f1[4101:4102]+f1[4126:4127]+f1[4175:4176]+\
    f1[4182:4183]+f1[4234:4235]+f1[4260:4261]+f1[4288:4289]+f1[4336:4337]+\
    f1[4364:4365]+f1[4369:4370]+f1[4374:4375]+f1[4437:4438]+f1[4440:4441]+\
    f1[4470:4471]+f1[4497:4498]+f1[4509:4510]+f1[4532:4533]+f1[4577:4578]+\
    f1[4598:4599]+f1[4700:4701]+f1[4760:4761]+f1[4801:4802]+f1[4810:4811]+\
    f1[4838:4839]+f1[4856:4857]+f1[4923:4924]+f1[4929:4930]+f1[4999:5000]+\
    f1[5040:5041]+f1[5065:5066]+f1[5080:5081]+f1[5118:5119]+f1[5126:5127]+\
    f1[5140:5141]+f1[5142:5143]+f1[5167:5168]+f1[5209:5210]+f1[5216:5217]+\
    f1[5367:5368]+f1[5401:5402]+f1[5414:5415]+f1[5423:5424]+f1[5489:5490]+\
    f1[5499:5500]+f1[5508:5509]+f1[5518:5519]+f1[5526:5527]+f1[5575:5576]+\
    f1[5584:5585]+f1[5599:5600]+f1[5607:5608]+f1[5623:5624]+f1[5658:5659]+\
    f1[5663:5664]+f1[5668:5669]+f1[5673:5674]+f1[5680:5681]+f1[5690:5691]+\
    f1[5702:5703]+f1[5726:5727]+f1[5732:5733]+f1[5766:5767]+f1[5800:5801]+\
    f1[5835:5836]+f1[5875:5876]+f1[5904:5905]+f1[6039:6040]+f1[6138:6139]+\
    f1[6153:6154]+f1[6160:6161]+f1[6179:6180]+f1[6222:6223]+f1[6303:6304]+\
    f1[6313:6314]+f1[6501:6502]+f1[6514:6515]+f1[6536:6537]+f1[6598:6599]+\
    f1[6605:6606]+f1[6625:6626]+f1[6633:6634]+f1[6637:6638]+f1[6694:6695]+\
    f1[6747:6748]+f1[6784:6785]+f1[6822:6823]+f1[6872:6873]+f1[6905:6906]+\
    f1[6935:6936]+f1[6945:6946]+f1[6966:6967]+f1[7047:7048]+f1[7056:7057]+\
    f1[7091:7092]+f1[7133:7134]+f1[7156:7157]+f1[7158:7159]+f1[7162:7163]+\
    f1[7170:7171]+f1[7173:7174]+f1[7293:7294]+f1[7400:7401]+f1[7454:7455]+\
    f1[7462:7463]+f1[7507:7508]+f1[7522:7523]+f1[7545:7546]+f1[7575:7576]+\
    f1[7591:7592]+f1[7621:7623]+f1[7629:7630]+f1[7638:7639]+f1[7699:7700]+\
    f1[7715:7716]+f1[7733:7734]+f1[7750:7751]+f1[7790:7791]+f1[7801:7802]+\
    f1[7811:7812]+f1[7855:7856]+f1[7868:7869]+f1[7891:7892]+f1[7915:7916]+\
    f1[7984:7985]+f1[8026:8027]+f1[8028:8029]+f1[8093:8094]+f1[8112:8113]+\
    f1[8120:8121]+f1[8191:8192]+f1[8198:8199]+f1[8237:8238]+f1[8366:8367]+\
    f1[8447:8448]+f1[8507:8508]+f1[8533:8534]+f1[8542:8543]+f1[8546:8547]+\
    f1[8585:8586]+f1[8589:8590]+f1[8594:8595]+f1[8600:8601]
face2Elements1 = f1[27:28]+f1[96:97]+f1[196:197]+f1[202:203]+f1[233:234]+\
    f1[246:247]+f1[252:253]+f1[286:287]+f1[297:298]+f1[310:311]+f1[316:317]+\
    f1[342:343]+f1[434:435]+f1[437:438]+f1[457:458]+f1[546:547]+f1[554:555]+\
    f1[561:562]+f1[690:691]+f1[714:715]+f1[717:718]+f1[732:733]+f1[741:742]+\
    f1[781:782]+f1[786:787]+f1[870:871]+f1[893:894]+f1[901:902]+f1[909:910]+\
    f1[938:939]+f1[945:946]+f1[953:954]+f1[973:974]+f1[993:994]+f1[1000:1001]+\
    f1[1029:1030]+f1[1071:1072]+f1[1089:1090]+f1[1125:1126]+f1[1183:1184]+\
    f1[1212:1213]+f1[1223:1224]+f1[1304:1305]+f1[1328:1329]+f1[1387:1388]+\
    f1[1421:1422]+f1[1534:1535]+f1[1562:1563]+f1[1568:1569]+f1[1594:1595]+\
    f1[1607:1608]+f1[1632:1633]+f1[1638:1639]+f1[1658:1659]+f1[1686:1687]+\
    f1[1706:1707]+f1[1786:1787]+f1[1856:1857]+f1[1943:1944]+f1[1976:1977]+\
    f1[1983:1984]+f1[1991:1992]+f1[2015:2016]+f1[2029:2030]+f1[2055:2056]+\
    f1[2082:2083]+f1[2087:2089]+f1[2095:2096]+f1[2114:2115]+f1[2118:2119]+\
    f1[2137:2138]+f1[2147:2148]+f1[2152:2153]+f1[2200:2201]+f1[2313:2314]+\
    f1[2332:2333]+f1[2440:2441]+f1[2456:2457]+f1[2471:2472]+f1[2478:2479]+\
    f1[2545:2546]+f1[2580:2581]+f1[2597:2598]+f1[2603:2604]+f1[2633:2635]+\
    f1[2738:2739]+f1[2979:2980]+f1[3091:3092]+f1[3110:3111]+f1[3139:3140]+\
    f1[3208:3209]+f1[3249:3250]+f1[3252:3253]+f1[3352:3353]+f1[3358:3359]+\
    f1[3373:3374]+f1[3393:3395]+f1[3423:3424]+f1[3455:3456]+f1[3481:3482]+\
    f1[3519:3520]+f1[3534:3535]+f1[3582:3583]+f1[3630:3631]+f1[3638:3639]+\
    f1[3640:3641]+f1[3716:3717]+f1[3747:3748]+f1[3850:3851]+f1[3868:3869]+\
    f1[3921:3923]+f1[3980:3981]+f1[4008:4009]+f1[4017:4018]+f1[4057:4058]+\
    f1[4177:4179]+f1[4184:4185]+f1[4255:4256]+f1[4272:4273]+f1[4340:4341]+\
    f1[4376:4377]+f1[4441:4442]+f1[4503:4504]+f1[4547:4549]+f1[4568:4569]+\
    f1[4635:4636]+f1[4656:4657]+f1[4666:4667]+f1[4709:4710]+f1[4716:4717]+\
    f1[4727:4728]+f1[4738:4739]+f1[4742:4743]+f1[4772:4773]+f1[4782:4783]+\
    f1[4837:4838]+f1[4910:4911]+f1[4959:4960]+f1[4989:4990]+f1[5236:5237]+\
    f1[5242:5243]+f1[5334:5335]+f1[5338:5339]+f1[5364:5365]+f1[5415:5416]+\
    f1[5501:5502]+f1[5535:5536]+f1[5560:5561]+f1[5572:5573]+f1[5651:5652]+\
    f1[5729:5730]+f1[5761:5762]+f1[5795:5796]+f1[5829:5830]+f1[5852:5853]+\
    f1[5928:5929]+f1[6032:6033]+f1[6085:6086]+f1[6261:6262]+f1[6323:6324]+\
    f1[6392:6393]+f1[6398:6399]+f1[6420:6421]+f1[6468:6469]+f1[6483:6484]+\
    f1[6485:6486]+f1[6557:6558]+f1[6559:6560]+f1[6565:6566]+f1[6576:6577]+\
    f1[6640:6641]+f1[6648:6649]+f1[6661:6662]+f1[6663:6664]+f1[6753:6754]+\
    f1[6804:6805]+f1[6818:6819]+f1[6850:6851]+f1[6919:6920]+f1[6936:6937]+\
    f1[6981:6982]+f1[7031:7032]+f1[7088:7089]+f1[7144:7145]+f1[7153:7154]+\
    f1[7178:7179]+f1[7194:7195]+f1[7209:7210]+f1[7274:7275]+f1[7361:7362]+\
    f1[7453:7454]+f1[7483:7484]+f1[7512:7513]+f1[7566:7567]+f1[7604:7605]+\
    f1[7611:7612]+f1[7634:7635]+f1[7667:7668]+f1[7669:7670]+f1[7727:7731]+\
    f1[7754:7755]+f1[7758:7759]+f1[7803:7804]+f1[7818:7819]+f1[7882:7883]+\
    f1[7908:7909]+f1[7928:7929]+f1[7935:7936]+f1[7947:7948]+f1[7966:7967]+\
    f1[7968:7969]+f1[7991:7992]+f1[8074:8075]+f1[8085:8086]+f1[8094:8095]+\
    f1[8097:8098]+f1[8359:8360]+f1[8388:8389]+f1[8406:8407]+f1[8489:8490]+\
    f1[8539:8540]+f1[8573:8574]
face3Elements1 = f1[13:14]+f1[56:57]+f1[109:110]+f1[120:121]+f1[137:138]+\
    f1[142:143]+f1[166:167]+f1[186:187]+f1[190:191]+f1[232:233]+f1[259:260]+\
    f1[263:264]+f1[329:330]+f1[346:347]+f1[401:402]+f1[421:422]+f1[466:467]+\
    f1[483:484]+f1[485:486]+f1[492:493]+f1[509:510]+f1[523:524]+f1[556:557]+\
    f1[607:608]+f1[616:617]+f1[641:642]+f1[645:646]+f1[648:649]+f1[651:652]+\
    f1[653:654]+f1[670:671]+f1[679:680]+f1[712:713]+f1[736:737]+f1[739:740]+\
    f1[755:756]+f1[760:761]+f1[779:780]+f1[804:805]+f1[808:809]+f1[855:858]+\
    f1[905:906]+f1[921:922]+f1[927:928]+f1[978:979]+f1[985:986]+f1[1023:1024]+\
    f1[1026:1027]+f1[1062:1063]+f1[1072:1073]+f1[1115:1116]+f1[1118:1119]+\
    f1[1165:1166]+f1[1255:1256]+f1[1257:1258]+f1[1288:1289]+f1[1301:1302]+\
    f1[1307:1308]+f1[1369:1370]+f1[1402:1403]+f1[1404:1405]+f1[1417:1418]+\
    f1[1432:1433]+f1[1434:1436]+f1[1450:1451]+f1[1489:1490]+f1[1528:1529]+\
    f1[1577:1578]+f1[1600:1601]+f1[1609:1610]+f1[1644:1645]+f1[1646:1647]+\
    f1[1653:1654]+f1[1657:1658]+f1[1684:1685]+f1[1724:1725]+f1[1729:1730]+\
    f1[1761:1762]+f1[1783:1784]+f1[1790:1791]+f1[1820:1821]+f1[1823:1824]+\
    f1[1833:1834]+f1[1836:1837]+f1[1851:1852]+f1[1858:1859]+f1[1925:1926]+\
    f1[1932:1933]+f1[1964:1965]+f1[1968:1969]+f1[1970:1972]+f1[1979:1980]+\
    f1[2017:2018]+f1[2034:2035]+f1[2046:2047]+f1[2078:2079]+f1[2164:2165]+\
    f1[2178:2179]+f1[2232:2233]+f1[2241:2242]+f1[2248:2249]+f1[2301:2302]+\
    f1[2307:2308]+f1[2320:2321]+f1[2358:2359]+f1[2377:2378]+f1[2388:2390]+\
    f1[2402:2403]+f1[2406:2407]+f1[2443:2444]+f1[2448:2449]+f1[2468:2469]+\
    f1[2477:2478]+f1[2492:2493]+f1[2507:2508]+f1[2519:2520]+f1[2528:2529]+\
    f1[2550:2551]+f1[2561:2562]+f1[2628:2629]+f1[2699:2700]+f1[2746:2747]+\
    f1[2772:2773]+f1[2776:2777]+f1[2791:2792]+f1[2811:2812]+f1[2814:2815]+\
    f1[2824:2825]+f1[2826:2827]+f1[2831:2832]+f1[2842:2843]+f1[2937:2938]+\
    f1[2960:2962]+f1[2969:2970]+f1[2995:2996]+f1[3010:3011]+f1[3102:3103]+\
    f1[3113:3114]+f1[3137:3138]+f1[3149:3150]+f1[3153:3154]+f1[3227:3228]+\
    f1[3232:3233]+f1[3236:3237]+f1[3282:3283]+f1[3316:3318]+f1[3334:3335]+\
    f1[3349:3350]+f1[3353:3354]+f1[3368:3369]+f1[3377:3378]+f1[3384:3385]+\
    f1[3387:3388]+f1[3395:3396]+f1[3399:3400]+f1[3402:3403]+f1[3428:3429]+\
    f1[3458:3459]+f1[3541:3542]+f1[3564:3565]+f1[3567:3568]+f1[3576:3577]+\
    f1[3585:3586]+f1[3594:3595]+f1[3626:3627]+f1[3650:3651]+f1[3666:3668]+\
    f1[3684:3685]+f1[3718:3719]+f1[3752:3753]+f1[3764:3765]+f1[3806:3807]+\
    f1[3831:3833]+f1[3898:3899]+f1[4151:4152]+f1[4200:4202]+f1[4246:4247]+\
    f1[4267:4268]+f1[4318:4319]+f1[4354:4355]+f1[4391:4392]+f1[4396:4397]+\
    f1[4411:4412]+f1[4449:4450]+f1[4456:4457]+f1[4498:4499]+f1[4536:4537]+\
    f1[4542:4543]+f1[4564:4565]+f1[4592:4593]+f1[4624:4625]+f1[4630:4631]+\
    f1[4645:4646]+f1[4665:4666]+f1[4671:4673]+f1[4702:4703]+f1[4739:4740]+\
    f1[4747:4748]+f1[4750:4751]+f1[4765:4766]+f1[4779:4780]+f1[4818:4819]+\
    f1[4822:4823]+f1[4894:4895]+f1[4898:4899]+f1[4938:4939]+f1[4965:4966]+\
    f1[4969:4970]+f1[4983:4984]+f1[5046:5047]+f1[5101:5102]+f1[5109:5110]+\
    f1[5130:5131]+f1[5179:5180]+f1[5231:5232]+f1[5237:5238]+f1[5251:5252]+\
    f1[5318:5319]+f1[5359:5360]+f1[5373:5374]+f1[5422:5423]+f1[5445:5446]+\
    f1[5463:5465]+f1[5523:5524]+f1[5525:5526]+f1[5533:5534]+f1[5537:5538]+\
    f1[5571:5572]+f1[5611:5612]+f1[5633:5634]+f1[5640:5641]+f1[5644:5645]+\
    f1[5660:5661]+f1[5669:5670]+f1[5676:5677]+f1[5678:5679]+f1[5698:5699]+\
    f1[5710:5711]+f1[5780:5781]+f1[5783:5784]+f1[5803:5804]+f1[5806:5807]+\
    f1[5878:5879]+f1[5880:5881]+f1[5900:5901]+f1[5934:5935]+f1[5980:5981]+\
    f1[5988:5989]+f1[6148:6150]+f1[6158:6159]+f1[6177:6178]+f1[6196:6198]+\
    f1[6202:6203]+f1[6218:6219]+f1[6267:6268]+f1[6336:6337]+f1[6344:6345]+\
    f1[6388:6389]+f1[6444:6445]+f1[6526:6527]+f1[6532:6533]+f1[6550:6551]+\
    f1[6568:6569]+f1[6574:6575]+f1[6578:6579]+f1[6584:6585]+f1[6594:6595]+\
    f1[6602:6603]+f1[6606:6607]+f1[6670:6671]+f1[6696:6697]+f1[6716:6717]+\
    f1[6737:6738]+f1[6748:6749]+f1[6836:6837]+f1[6902:6903]+f1[6996:6997]+\
    f1[7137:7138]+f1[7150:7151]+f1[7154:7155]+f1[7193:7194]+f1[7276:7277]+\
    f1[7302:7304]+f1[7343:7344]+f1[7345:7346]+f1[7427:7428]+f1[7502:7503]+\
    f1[7542:7543]+f1[7555:7556]+f1[7563:7564]+f1[7572:7573]+f1[7577:7578]+\
    f1[7584:7585]+f1[7631:7632]+f1[7653:7654]+f1[7659:7660]+f1[7681:7682]+\
    f1[7753:7754]+f1[7756:7757]+f1[7760:7761]+f1[7778:7780]+f1[7781:7782]+\
    f1[7791:7792]+f1[7853:7854]+f1[7877:7878]+f1[7889:7890]+f1[7925:7926]+\
    f1[7967:7968]+f1[7982:7983]+f1[8021:8022]+f1[8037:8038]+f1[8102:8103]+\
    f1[8140:8141]+f1[8183:8184]+f1[8188:8189]+f1[8192:8193]+f1[8210:8211]+\
    f1[8217:8218]+f1[8225:8226]+f1[8234:8235]+f1[8313:8314]+f1[8365:8366]+\
    f1[8399:8400]+f1[8429:8430]+f1[8452:8453]+f1[8458:8459]+f1[8484:8485]+\
    f1[8493:8494]+f1[8509:8510]+f1[8568:8569]+f1[8575:8576]+f1[8579:8580]+\
    f1[8582:8583]+f1[8590:8592]+f1[8599:8600]
face4Elements1 = f1[14:15]+f1[105:106]+f1[152:153]+f1[154:155]+f1[187:188]+\
    f1[214:215]+f1[222:223]+f1[251:252]+f1[277:278]+f1[301:302]+f1[328:329]+\
    f1[336:337]+f1[359:360]+f1[372:373]+f1[380:381]+f1[533:534]+f1[555:556]+\
    f1[631:632]+f1[710:711]+f1[725:726]+f1[729:730]+f1[789:790]+f1[814:815]+\
    f1[826:827]+f1[858:859]+f1[877:878]+f1[889:890]+f1[894:895]+f1[947:948]+\
    f1[967:968]+f1[986:987]+f1[995:997]+f1[1024:1025]+f1[1031:1032]+\
    f1[1074:1075]+f1[1088:1089]+f1[1108:1109]+f1[1149:1150]+f1[1156:1157]+\
    f1[1211:1212]+f1[1235:1236]+f1[1248:1249]+f1[1252:1254]+f1[1275:1276]+\
    f1[1321:1322]+f1[1337:1339]+f1[1355:1357]+f1[1372:1373]+f1[1374:1375]+\
    f1[1415:1416]+f1[1431:1432]+f1[1438:1439]+f1[1459:1460]+f1[1527:1528]+\
    f1[1530:1531]+f1[1573:1574]+f1[1582:1583]+f1[1586:1587]+f1[1596:1597]+\
    f1[1650:1651]+f1[1652:1653]+f1[1662:1663]+f1[1709:1710]+f1[1742:1743]+\
    f1[1785:1786]+f1[1801:1802]+f1[1815:1816]+f1[1835:1836]+f1[1837:1838]+\
    f1[1840:1841]+f1[1865:1866]+f1[1870:1871]+f1[1875:1876]+f1[1926:1927]+\
    f1[1936:1937]+f1[1963:1964]+f1[2019:2020]+f1[2045:2046]+f1[2053:2054]+\
    f1[2057:2058]+f1[2142:2143]+f1[2179:2180]+f1[2192:2193]+f1[2196:2197]+\
    f1[2201:2202]+f1[2218:2219]+f1[2220:2221]+f1[2222:2223]+f1[2261:2262]+\
    f1[2331:2332]+f1[2367:2368]+f1[2426:2427]+f1[2449:2450]+f1[2453:2454]+\
    f1[2499:2500]+f1[2509:2510]+f1[2526:2527]+f1[2620:2621]+f1[2644:2645]+\
    f1[2657:2658]+f1[2659:2662]+f1[2717:2718]+f1[2722:2723]+f1[2725:2726]+\
    f1[2744:2745]+f1[2748:2749]+f1[2781:2782]+f1[2841:2842]+f1[2907:2908]+\
    f1[2956:2957]+f1[2982:2983]+f1[3097:3098]+f1[3103:3104]+f1[3109:3110]+\
    f1[3250:3251]+f1[3263:3264]+f1[3270:3271]+f1[3342:3343]+f1[3391:3392]+\
    f1[3398:3399]+f1[3424:3425]+f1[3433:3434]+f1[3444:3445]+f1[3450:3452]+\
    f1[3453:3454]+f1[3467:3468]+f1[3474:3475]+f1[3485:3486]+f1[3507:3508]+\
    f1[3517:3518]+f1[3528:3529]+f1[3535:3536]+f1[3539:3540]+f1[3547:3548]+\
    f1[3553:3554]+f1[3569:3570]+f1[3631:3632]+f1[3634:3635]+f1[3751:3752]+\
    f1[3787:3788]+f1[3814:3815]+f1[3821:3822]+f1[3839:3840]+f1[3843:3844]+\
    f1[3851:3852]+f1[3877:3878]+f1[3880:3881]+f1[3946:3947]+f1[3993:3994]+\
    f1[3997:3998]+f1[4042:4043]+f1[4118:4119]+f1[4127:4128]+f1[4198:4199]+\
    f1[4269:4270]+f1[4304:4305]+f1[4313:4314]+f1[4386:4387]+f1[4432:4433]+\
    f1[4446:4447]+f1[4459:4460]+f1[4463:4464]+f1[4479:4480]+f1[4484:4485]+\
    f1[4491:4492]+f1[4502:4503]+f1[4506:4507]+f1[4512:4513]+f1[4562:4563]+\
    f1[4587:4588]+f1[4637:4638]+f1[4646:4647]+f1[4650:4651]+f1[4653:4654]+\
    f1[4660:4662]+f1[4684:4685]+f1[4696:4697]+f1[4724:4726]+f1[4748:4749]+\
    f1[4753:4754]+f1[4796:4797]+f1[4833:4834]+f1[4835:4836]+f1[4860:4861]+\
    f1[4866:4867]+f1[4868:4869]+f1[4880:4881]+f1[4884:4885]+f1[4901:4902]+\
    f1[4925:4926]+f1[4933:4934]+f1[4955:4956]+f1[5009:5010]+f1[5030:5031]+\
    f1[5095:5096]+f1[5102:5103]+f1[5107:5109]+f1[5123:5124]+f1[5166:5167]+\
    f1[5172:5173]+f1[5174:5175]+f1[5178:5179]+f1[5219:5221]+f1[5241:5242]+\
    f1[5247:5248]+f1[5363:5364]+f1[5381:5382]+f1[5390:5391]+f1[5398:5399]+\
    f1[5400:5401]+f1[5473:5474]+f1[5487:5488]+f1[5522:5523]+f1[5545:5546]+\
    f1[5555:5556]+f1[5570:5571]+f1[5605:5606]+f1[5612:5613]+f1[5632:5633]+\
    f1[5652:5653]+f1[5656:5657]+f1[5664:5665]+f1[5703:5704]+f1[5709:5710]+\
    f1[5763:5764]+f1[5840:5841]+f1[5850:5851]+f1[5856:5857]+f1[5872:5873]+\
    f1[5903:5904]+f1[5906:5908]+f1[6014:6015]+f1[6114:6115]+f1[6207:6208]+\
    f1[6284:6285]+f1[6342:6343]+f1[6391:6392]+f1[6394:6396]+f1[6413:6414]+\
    f1[6453:6454]+f1[6475:6476]+f1[6504:6505]+f1[6506:6507]+f1[6508:6509]+\
    f1[6538:6539]+f1[6544:6546]+f1[6577:6578]+f1[6592:6593]+f1[6604:6605]+\
    f1[6617:6618]+f1[6634:6635]+f1[6636:6637]+f1[6639:6640]+f1[6659:6660]+\
    f1[6689:6690]+f1[6711:6712]+f1[6774:6775]+f1[6841:6842]+f1[6904:6905]+\
    f1[6937:6938]+f1[6964:6965]+f1[7005:7006]+f1[7045:7046]+f1[7075:7076]+\
    f1[7237:7238]+f1[7289:7290]+f1[7355:7356]+f1[7357:7359]+f1[7386:7387]+\
    f1[7456:7457]+f1[7517:7518]+f1[7588:7589]+f1[7592:7593]+f1[7596:7597]+\
    f1[7612:7613]+f1[7614:7615]+f1[7624:7625]+f1[7637:7638]+f1[7646:7647]+\
    f1[7649:7650]+f1[7664:7665]+f1[7684:7685]+f1[7697:7698]+f1[7703:7704]+\
    f1[7717:7718]+f1[7719:7721]+f1[7762:7763]+f1[7780:7781]+f1[7783:7784]+\
    f1[7792:7793]+f1[7805:7806]+f1[7902:7903]+f1[7926:7927]+f1[8034:8035]+\
    f1[8129:8130]+f1[8163:8164]+f1[8180:8181]+f1[8243:8244]+f1[8391:8392]+\
    f1[8441:8442]+f1[8465:8466]+f1[8492:8493]+f1[8506:8507]+f1[8519:8520]+\
    f1[8536:8537]+f1[8548:8549]+f1[8576:8577]+f1[8578:8579]+f1[8580:8581]+\
    f1[8593:8594]+f1[8596:8597]+f1[8598:8599]
a.Surface(face1Elements=face1Elements1, face2Elements=face2Elements1, 
    face3Elements=face3Elements1, face4Elements=face4Elements1, name='plug')




a = mdb.models['Knee'].rootAssembly
n1 = a.instances['plug6-1'].nodes
nodes1 = n1[24:25]+n1[43:44]+n1[65:68]+n1[69:70]+n1[150:151]+n1[175:176]+\
    n1[201:202]+n1[204:206]+n1[237:239]+n1[292:294]+n1[345:347]+n1[485:487]+\
    n1[554:555]+n1[658:660]+n1[662:663]+n1[766:767]+n1[780:781]+n1[823:825]+\
    n1[831:832]+n1[833:836]+n1[837:838]+n1[853:857]+n1[867:868]+n1[885:886]+\
    n1[890:892]+n1[924:925]+n1[953:954]+n1[1007:1008]+n1[1012:1013]+\
    n1[1066:1067]+n1[1081:1083]+n1[1098:1101]+n1[1109:1110]+n1[1129:1130]+\
    n1[1146:1147]+n1[1150:1152]+n1[1153:1154]+n1[1166:1167]+n1[1176:1177]+\
    n1[1339:1342]+n1[1354:1355]+n1[1363:1365]+n1[1368:1369]+n1[1376:1377]+\
    n1[1381:1383]+n1[1417:1418]+n1[1420:1421]+n1[1443:1444]+n1[1462:1463]+\
    n1[1471:1472]+n1[1474:1475]+n1[1518:1520]+n1[1547:1548]+n1[1561:1562]+\
    n1[1568:1569]+n1[1583:1584]+n1[1593:1594]+n1[1601:1602]+n1[1611:1612]+\
    n1[1616:1617]+n1[1623:1624]+n1[1626:1627]+n1[1647:1648]+n1[1649:1650]+\
    n1[1652:1653]+n1[1659:1660]+n1[1668:1669]+n1[1722:1725]+n1[1732:1733]+\
    n1[1734:1735]+n1[1747:1748]+n1[1750:1751]+n1[1759:1760]+n1[1773:1774]+\
    n1[1825:1827]
a.Set(nodes=nodes1, name='top')



a = mdb.models['Knee'].rootAssembly
f1 = a.instances['plug6-1'].elements
face1Elements1 = f1[170:171]+f1[361:362]+f1[418:419]+f1[429:430]+f1[490:491]+\
    f1[497:498]+f1[508:509]+f1[535:536]+f1[569:570]+f1[770:771]+f1[1454:1455]+\
    f1[1488:1489]+f1[1505:1506]+f1[1880:1881]+f1[2259:2260]+f1[2626:2627]+\
    f1[3046:3047]+f1[3201:3202]+f1[3546:3547]+f1[3573:3574]+f1[3891:3892]+\
    f1[4248:4250]+f1[4427:4428]+f1[5304:5305]+f1[5432:5433]+f1[5686:5687]+\
    f1[5700:5701]+f1[5951:5952]+f1[6174:6175]+f1[6239:6240]+f1[6290:6291]+\
    f1[6310:6311]+f1[6759:6760]+f1[6762:6763]+f1[7003:7004]+f1[7367:7368]+\
    f1[8030:8031]+f1[8256:8257]+f1[8383:8384]
face2Elements1 = f1[20:21]+f1[593:594]+f1[777:778]+f1[1037:1038]+f1[1233:1234]+\
    f1[1243:1244]+f1[1272:1273]+f1[1383:1384]+f1[1670:1671]+f1[1896:1897]+\
    f1[1902:1903]+f1[1942:1943]+f1[2150:2151]+f1[2847:2848]+f1[3037:3038]+\
    f1[3070:3071]+f1[3897:3898]+f1[3941:3942]+f1[4447:4448]+f1[4475:4476]+\
    f1[4493:4494]+f1[4559:4560]+f1[4889:4890]+f1[4962:4963]+f1[4992:4993]+\
    f1[5115:5116]+f1[5505:5506]+f1[6907:6908]+f1[6995:6996]+f1[7012:7013]+\
    f1[7470:7471]+f1[8017:8018]+f1[8249:8250]+f1[8405:8406]
face3Elements1 = f1[407:408]+f1[518:519]+f1[530:531]+f1[570:571]+f1[597:598]+\
    f1[749:750]+f1[797:798]+f1[1126:1127]+f1[1287:1288]+f1[1664:1666]+\
    f1[1702:1703]+f1[1843:1844]+f1[2069:2070]+f1[2198:2199]+f1[2283:2284]+\
    f1[2319:2320]+f1[2806:2807]+f1[2873:2874]+f1[3155:3156]+f1[3313:3314]+\
    f1[3360:3361]+f1[4367:4368]+f1[4429:4430]+f1[4517:4518]+f1[4572:4573]+\
    f1[4628:4629]+f1[5072:5073]+f1[5138:5139]+f1[5207:5208]+f1[5227:5228]+\
    f1[5294:5295]+f1[5457:5458]+f1[5467:5468]+f1[5484:5485]+f1[5924:5925]+\
    f1[6048:6049]+f1[6405:6406]+f1[6419:6420]+f1[6595:6596]+f1[6599:6601]+\
    f1[6687:6688]+f1[6725:6726]+f1[6985:6986]+f1[7028:7029]+f1[7673:7674]+\
    f1[7676:7677]+f1[8045:8046]
face4Elements1 = f1[363:364]+f1[551:552]+f1[759:760]+f1[1018:1019]+\
    f1[1035:1036]+f1[1344:1345]+f1[1352:1353]+f1[1401:1402]+f1[1508:1509]+\
    f1[1538:1539]+f1[1654:1656]+f1[2181:2182]+f1[2318:2319]+f1[2723:2724]+\
    f1[3090:3091]+f1[3196:3197]+f1[3309:3310]+f1[3318:3319]+f1[3544:3545]+\
    f1[3606:3607]+f1[3609:3610]+f1[4448:4449]+f1[4576:4577]+f1[4596:4597]+\
    f1[4608:4609]+f1[4638:4639]+f1[4669:4670]+f1[4843:4844]+f1[5088:5089]+\
    f1[5133:5134]+f1[5145:5146]+f1[5308:5309]+f1[5431:5432]+f1[5600:5601]+\
    f1[5685:5686]+f1[6180:6181]+f1[6306:6307]+f1[6643:6644]+f1[6726:6727]+\
    f1[6765:6766]+f1[7094:7095]+f1[7388:7389]+f1[7751:7752]+f1[8019:8020]+\
    f1[8565:8566]
a.Surface(face1Elements=face1Elements1, face2Elements=face2Elements1, 
    face3Elements=face3Elements1, face4Elements=face4Elements1, 
    name='ptop_surf')

#----------------------------------------------------------------------------------------
# 
#----------------------------------------------------------------------------------------

#----------------------------------------------------------------------------------------
# Boundary Conditions - part 2
#----------------------------------------------------------------------------------------




a = mdb.models['Knee'].rootAssembly
region1=a.surfaces['ptop_surf']
a = mdb.models['Knee'].rootAssembly
region2=a.surfaces['SF_TCART_MED_SELF_CONTACT']
mdb.models['Knee'].SurfaceToSurfaceContactStd(name='ptoptcart', 
    createStepName='Move', master=region1, slave=region2, sliding=FINITE, 
    thickness=ON, interactionProperty='IntProp-1', adjustMethod=NONE, 
    initialClearance=OMIT, datumAxis=None, clearanceRegion=None)

Mod.DisplacementBC(amplitude=UNSET, createStepName=
    'displacement', distributionType=UNIFORM, fieldName='', fixed=OFF,
    localCsys=
    Mod.rootAssembly.instances['plug6-1'].datums[258],
    name='BC-2', region=Mod.rootAssembly.sets['top'], u1=
    UNSET, u2=UNSET, u3=dispQuant+0.5, ur1=UNSET, ur2=UNSET, ur3=UNSET)

Mod.boundaryConditions['BC-2'].deactivate('relax')
mdb.models['Knee'].boundaryConditions['Displace_fem'].move('Move', 'relax')
mdb.models['Knee'].boundaryConditions['Displace_fem'].move('relax', 
    'displacement')
mdb.models['Knee'].boundaryConditions['Displace_fem'].setValues(u3=0.0)
mdb.models['Knee'].boundaryConditions['Displace_fem'].setValuesInStep(
    stepName='Move', u3=FREED)
mdb.models['Knee'].boundaryConditions['Displace_fem'].setValuesInStep(
    stepName='Move', u3=-2.0)
#----------------------------------------------------------------------------------------
# Graft Interactions
#----------------------------------------------------------------------------------------
#cartilage + plug contact
Mod.ContactProperty('cart_plug')
Mod.interactionProperties['cart_plug'].TangentialBehavior(
    dependencies=0, directionality=ISOTROPIC, elasticSlipStiffness=None,
    formulation=PENALTY, fraction=0.005, maximumElasticSlip=FRACTION,
    pressureDependency=OFF, shearStressLimit=None, slipRateDependency=OFF,
    table=((0.1, ), ), temperatureDependency=OFF)
Mod.interactionProperties['cart_plug'].NormalBehavior(
    allowSeparation=ON, constraintEnforcementMethod=DEFAULT,
    pressureOverclosure=HARD)
Mod.SurfaceToSurfaceContactStd(adjustMethod=NONE,
    clearanceRegion=None, createStepName='Initial', datumAxis=None,
    initialClearance=OMIT, interactionProperty='cart_plug', master=
    Mod.rootAssembly.surfaces['SF_FCART_WITH_CYL_Y_10'], name=
    'cart-bone', slave=Mod.rootAssembly.surfaces['plug'],
    sliding=FINITE, thickness=ON)
Mod.ContactProperty('bone-plug')
Mod.interactionProperties['bone-plug'].NormalBehavior(
    allowSeparation=ON, constraintEnforcementMethod=DEFAULT,
    pressureOverclosure=HARD)
Mod.interactionProperties['bone-plug'].TangentialBehavior(
    dependencies=0, directionality=ISOTROPIC, elasticSlipStiffness=None,
    formulation=PENALTY, fraction=0.005, maximumElasticSlip=FRACTION,
    pressureDependency=OFF, shearStressLimit=None, slipRateDependency=OFF,
    table=((0.2, ), ), temperatureDependency=OFF)

# contact bone+plug

Mod.SurfaceToSurfaceContactStd(adjustMethod=NONE,
    clearanceRegion=None, createStepName='Initial', datumAxis=None,
    initialClearance=OMIT, interactionProperty='bone-plug', master=
    Mod.rootAssembly.surfaces['SF_FEMUR_WITH_CYL_Y_10'], name=
    'bone-bone', slave=Mod.rootAssembly.surfaces['plug'],
    sliding=FINITE, thickness=ON)

#----------------------------------------------------------------------------------------
# 
#----------------------------------------------------------------------------------------


#----------------------------------------------------------------------------------------
# Material Properties
#----------------------------------------------------------------------------------------



Mod.materials['PM_FCART'].Elastic(table=((6.0,0.46),))
Mod.materials['PM_TCART_LAT'].Elastic(table=((6.0,0.46),))
Mod.materials['PM_TCART_MED'].Elastic(table=((6.0,0.46),))


for mat in Mod.materials.keys():
    myMat = Mod.materials[mat]
    if mat.endswith('FEMUR_GS'):
        rho = int(myMat.density.table[0][0])
        if rho<1:rho=1
        E =rho*0.5
        nu = 0.3
        del myMat.elastic
        myMat.Elastic(table=((E, nu), ))
    if mat.endswith('TIBIA_GS'):
        rho = int(myMat.density.table[0][0])
        if rho<1:rho=1
        E =rho*0.5
        nu = 0.3
        del myMat.elastic
        myMat.Elastic(table=((E, nu), ))
    if mat.endswith('PLUG6_GS'):
        rho = int(myMat.density.table[0][0])
        if rho<1:rho=1
        E =rho*0.5
        nu = 0.3
        del myMat.elastic
        myMat.Elastic(table=((E, nu), ))

Mod.materials['PMGS0_PLUG6_GS'].elastic.setValues(table=((6.0,
    0.46), ))
# mesh


mdb.models['Knee'].interactions['Int-6'].move('Initial', 'Move')
mdb.models['Knee'].interactions['Int-5'].move('Initial', 'Move')


Name ='Knee1_y+10' 
Mod.rootAssembly.regenerate()		
myJob = mdb.Job(name=Name, model='Knee')
myJob.setValues(memory=90, memoryUnits=PERCENTAGE)
myJob.setValues(numCpus=20,numDomains=20,multiprocessingMode=THREADS,numGPUs=1)
mdb.saveAs(pathName=Name + '.cae')
mdb.jobs[Name].writeInput()
print >> sys.__stdout__, "setup complete. running model"
mdb.jobs[Name].submit()
