import geometry_calculation as cal
import fit_skspatial as fit
import matplot_show as mat
import numpy as np
import math
from threading import Thread

#quater 1 
pos_list_x = []
pos_list_y = []
pos_list_x.append([[-145.26776123046875, -90.40972900390625, 572.4722900390625], [-135.3714141845703, -88.98101043701172, 572.888916015625], [-125.26659393310547, -87.56026458740234, 573.75], [-115.17240142822266, -86.34642028808594, 574.2776489257812], [-105.11576843261719, -84.88916015625, 574.6666259765625], [-95.26589965820312, -83.76922607421875, 575.7778930664062], [-85.22811889648438, -82.41271209716797, 576.25], [-75.2573013305664, -81.20286560058594, 577.0], [-65.34465026855469, -79.93296813964844, 577.77783203125], [-55.38014221191406, -78.67484283447266, 578.4166870117188], [-45.52643585205078, -77.5286636352539, 579.9722290039062], [-35.670684814453125, -76.3949203491211, 581.2499389648438], [-25.75136375427246, -75.16297912597656, 582.0555419921875], [-15.748530387878418, -73.83399963378906, 582.6111450195312], [-5.868849754333496, -72.97172546386719, 585.361083984375], [4.049104690551758, -71.673583984375, 586.0556030273438], [14.038990020751953, -70.4884262084961, 586.72216796875], [23.916364669799805, -69.29815673828125, 587.5555419921875], [33.9287109375, -68.24417877197266, 588.9722900390625], [43.92490005493164, -66.9923095703125, 589.6943969726562], [53.87577438354492, -65.74057006835938, 589.7500610351562], [63.837825775146484, -64.5618896484375, 590.7777099609375], [73.73754119873047, -63.419700622558594, 591.611083984375], [83.6598129272461, -62.088462829589844, 591.7498779296875], [93.71131134033203, -60.938663482666016, 592.41650390625], [103.81930541992188, -59.7697639465332, 593.2777709960938], [113.6833267211914, -58.439937591552734, 593.1666259765625], [123.94425201416016, -57.373130798339844, 594.4443969726562], [134.12591552734375, -56.189598083496094, 595.6945190429688], [144.11129760742188, -54.94530487060547, 596.22216796875], [154.15281677246094, -53.70021438598633, 596.4444580078125], [164.50784301757812, -52.6021842956543, 597.9166870117188], [174.69375610351562, -51.47886276245117, 598.8333129882812], [184.8940887451172, -50.28243637084961, 599.6666870117188], [195.15916442871094, -49.03261184692383, 600.5556030273438], [205.1661834716797, -47.80826950073242, 600.6389770507812], [215.68936157226562, -46.69007110595703, 602.1944580078125], [225.9807586669922, -45.40730285644531, 603.2222900390625], [236.20404052734375, -44.221824645996094, 604.0833129882812], [246.4501495361328, -43.07304382324219, 604.8333740234375]])
pos_list_y.append([[65.13286590576172, -166.3097381591797, 603.8333740234375], [64.42239379882812, -158.53956604003906, 602.8611450195312], [63.52341842651367, -149.43943786621094, 601.8054809570312], [62.39866638183594, -140.1688690185547, 600.0278930664062], [61.36592102050781, -131.27059936523438, 599.5556030273438], [60.460235595703125, -123.09403228759766, 597.3055419921875], [59.27865219116211, -113.98872375488281, 595.9166870117188], [58.265830993652344, -105.12931823730469, 595.4722900390625], [57.1263427734375, -96.07537841796875, 593.8889770507812], [56.06396484375, -88.00556182861328, 592.3054809570312], [55.14773178100586, -79.129638671875, 591.8887939453125], [54.118263244628906, -70.14170837402344, 590.5554809570312], [53.10504913330078, -61.15058517456055, 589.3056030273438], [52.24386215209961, -53.23252868652344, 589.0833740234375], [51.23432159423828, -44.340187072753906, 587.8612060546875], [50.30255126953125, -35.417991638183594, 586.9722900390625], [49.29393768310547, -26.511775970458984, 586.5277709960938], [48.32667922973633, -18.404205322265625, 585.4166870117188], [47.283546447753906, -9.436016082763672, 583.8055419921875], [46.23724365234375, -0.5090945363044739, 582.3888549804688], [45.21452331542969, 8.45670223236084, 580.72216796875], [44.315338134765625, 16.398378372192383, 580.0], [43.305824279785156, 25.3635196685791, 577.861083984375], [42.35308074951172, 34.25904846191406, 576.4445190429688], [41.28460693359375, 43.29798889160156, 574.861083984375], [40.3313102722168, 51.20464324951172, 573.3611450195312], [39.390960693359375, 60.195037841796875, 572.6945190429688], [38.35649490356445, 69.04060363769531, 571.0], [37.41193389892578, 78.08726501464844, 570.5276489257812], [36.42142868041992, 85.96956634521484, 569.1666259765625], [35.538063049316406, 94.9980239868164, 568.72216796875], [34.499412536621094, 104.01982879638672, 567.6943969726562], [33.456825256347656, 112.98518371582031, 565.8612060546875], [32.498756408691406, 121.07634735107422, 564.97216796875], [31.46256446838379, 129.89801025390625, 562.9445190429688], [30.428180694580078, 138.91213989257812, 561.5556030273438], [29.446308135986328, 147.97276306152344, 560.5833740234375], [28.54593849182129, 156.22036743164062, 560.2777709960938], [27.56898307800293, 165.02328491210938, 557.5277709960938], [26.517314910888672, 173.78260803222656, 555.6943969726562]])

# 170 7 175
pos_list_x.append([[-125.81884765625, -12.099358558654785, 569.7777099609375], [-106.08080291748047, -16.67287254333496, 568.5000610351562], [-86.61428833007812, -21.14806365966797, 567.3611450195312], [-67.4115982055664, -25.625469207763672, 566.9165649414062], [-47.88935852050781, -30.2232723236084, 565.9166870117188], [-28.51447868347168, -34.78011703491211, 565.8888549804688], [-9.03559684753418, -39.360321044921875, 564.8055419921875], [10.493881225585938, -43.853904724121094, 564.5555419921875], [30.015666961669922, -48.392372131347656, 563.75], [49.46930694580078, -52.937870025634766, 562.6389770507812], [69.10281372070312, -57.536224365234375, 561.77783203125], [88.71510314941406, -62.165035247802734, 560.6389770507812], [108.35086059570312, -66.69110107421875, 558.97216796875], [128.09352111816406, -71.28314971923828, 557.9166259765625], [147.81817626953125, -76.09445190429688, 556.638916015625], [167.74240112304688, -80.74109649658203, 555.4166259765625], [187.75758361816406, -85.22538757324219, 554.0555419921875], [207.82008361816406, -90.29429626464844, 553.0833740234375], [227.64605712890625, -94.82422637939453, 552.0277709960938], [247.63955688476562, -99.0279541015625, 550.8056640625]])
pos_list_y.append([[47.54566955566406, -154.77853393554688, 577.6388549804688], [51.173316955566406, -138.42698669433594, 575.4445190429688], [55.03704071044922, -120.84405517578125, 572.6943359375], [58.75630569458008, -104.29730987548828, 570.0277709960938], [62.540435791015625, -86.75029754638672, 566.8056030273438], [66.25807189941406, -70.28897857666016, 564.3055419921875], [69.99114227294922, -52.831764221191406, 560.4166870117188], [73.7608871459961, -36.296024322509766, 557.9722900390625], [77.64685821533203, -18.916240692138672, 555.111083984375], [81.35604095458984, -2.402902126312256, 551.9445190429688], [85.32980346679688, 15.059367179870605, 549.75], [88.87837982177734, 31.5550537109375, 546.8056030273438], [92.82293701171875, 49.0435905456543, 543.1666870117188], [96.72700500488281, 65.66939544677734, 540.4722900390625], [100.86944580078125, 83.33761596679688, 537.7223510742188], [104.4391098022461, 99.85038757324219, 534.3889770507812], [108.27740478515625, 117.37361907958984, 530.6112060546875], [112.1431884765625, 134.13230895996094, 527.9722900390625], [116.0648422241211, 151.70211791992188, 524.2222900390625], [120.03005981445312, 168.3905792236328, 520.6945190429688]])

#quater 2
pos_list_x.append([[-223.72482438520953, 14.664756601507014, 516.4545842950994], [-212.8284287886186, 12.853798172690652, 518.727294921875], [-200.4605379971591, 10.303777001120828, 516.2727716619319], [-190.15715443004262, 9.368941046974875, 516.1818514737216], [-179.01400618119672, 7.1808178641579365, 515.4545953924006], [-166.88025595925072, 4.9784040451049805, 512.5454600941051], [-156.27254832874644, 3.1316746798428623, 513.7272782759233], [-145.4845164905895, 1.5852895704182712, 513.909102006392], [-135.07093811035156, -0.740422861142592, 515.5454933860085], [-123.48916417902166, -2.440637929873033, 513.9091075550426], [-112.5112436467951, -4.299935947765004, 514.0000166459517], [-101.27823500199752, -5.9897917834195225, 512.8181818181819], [-90.69916326349431, -7.709981268102473, 514.1818292791194], [-78.85180455988103, -9.678155378861861, 511.72728937322444], [-68.08224626020952, -11.558345534584738, 513.0909146395596], [-57.50596063787287, -13.362645062533291, 511.1818348277699], [-47.20833240855824, -15.374727335843174, 511.4545621004972], [-35.36900641701438, -17.240079532970082, 511.9090936834162], [-23.951318740844727, -19.553007472645152, 511.90910200639206], [-13.553442868319424, -21.15539446744052, 509.6364024769176], [-2.197128079154275, -23.11673788590865, 512.2727300470525], [9.83727333762429, -24.329965591430664, 511.81819291548294], [20.291114980524238, -26.28847191550515, 510.9091242009943], [30.300920139659535, -28.289832548661664, 510.90911865234375], [42.44321406971324, -29.96680485118519, 510.6364052512429], [52.47058764371005, -31.887311241843484, 509.3636668812145], [63.75147732821378, -33.84146638350053, 508.7272976962003], [74.17067163640803, -35.60500439730558, 507.6363913796165], [86.1966878717596, -37.80212471701882, 506.3636641068892], [97.13956243341619, -40.35365295410156, 507.72730047052556], [108.501220703125, -41.83730558915572, 507.1818486993963], [119.42005434903231, -44.15372016213157, 507.45457597212356], [131.60289278897372, -46.214238600297406, 504.9090936834162], [142.23119701038706, -47.72700465809215, 504.8181818181818], [153.23826321688566, -50.334252097389914, 502.00004161487925], [164.37351434881037, -51.87506866455078, 502.2727577903054], [176.81549349698153, -53.96910962191495, 500.6364024769176], [188.24004433371803, -56.05161493474787, 501.72730879350144], [199.1217998157848, -57.90837027809837, 502.0909451571378], [209.02272727272728, -59.46290449662642, 498.63637473366475]])
pos_list_y.append([[-30.556568839333274, -210.13726529208097, 535.7272727272727], [-29.657855987548828, -203.19285583496094, 537.0000055486506], [-28.42315032265403, -193.44156022505328, 535.0909146395596], [-26.687464800747957, -183.72758900035512, 532.636396928267], [-24.83668153936213, -173.66246448863637, 530.818192915483], [-23.455766504461113, -164.63912963867188, 528.545487837358], [-21.469953363591973, -155.66027415882456, 526.7272727272727], [-19.895718834616922, -146.87353099476206, 527.0909090909091], [-18.250511342828926, -137.23480502041903, 525.4545787464489], [-16.83088632063432, -128.41248390891334, 525.6363858309659], [-16.176783735101875, -119.71410023082386, 525.0000499378551], [-14.38658566908403, -110.6294735995206, 523.1818292791194], [-12.785127639770508, -100.81980063698508, 522.1818181818181], [-11.347636396234686, -92.3039814342152, 521.1818237304688], [-9.290159658952193, -83.37808366255327, 520.3636752041904], [-8.314043391834606, -74.28409229625355, 518.1818348277699], [-6.565093430605802, -64.45350820367986, 515.7273282137784], [-5.310557842254639, -54.867405284534804, 514.7273171164773], [-3.7494563406163994, -46.67049789428711, 513.2727272727273], [-2.2004946903748945, -38.14389350197532, 513.0], [-0.6529891341924667, -28.195907419378106, 512.0000055486506], [1.1988504772836512, -18.950396971269086, 512.2727328213779], [2.5731865059245718, -11.015667481855912, 509.2727577903054], [4.7102744362571025, -1.6467880335721103, 507.90912142666906], [5.481837576085871, 7.696621851487593, 508.5454767400568], [7.148733139038086, 16.384074731306598, 507.0909396084872], [8.018201567909934, 25.404599970037285, 504.18182650479406], [9.950992410833186, 34.976787913929336, 502.0000360662287], [11.40577220916748, 44.13046438043768, 499.45454822887075], [13.073738271539861, 53.2087270563299, 499.90909090909093], [14.696308656172318, 62.39981738003817, 498.72728105024856], [15.875774990428578, 71.31199645996094, 498.000019420277], [17.97720042142001, 81.51647741144353, 498.0000305175781], [18.947885166515004, 89.39267245205966, 495.90912142666906], [19.801251151344992, 98.14247547496448, 494.81821233575994], [21.688156821511008, 107.87322651256214, 493.63639415394175], [23.64418706026944, 116.32197015935725, 491.0000083229759], [24.53665455904874, 126.10987715287642, 491.0909312855114], [26.123221137306906, 134.30649636008522, 487.5454850630327], [27.702950911088422, 143.22905800559303, 486.90912142666906]])

pos_list_x.append([[-0.18534445762634277, 0.03042885661125183, 0.6669723391532898], [-0.17347539961338043, 0.02982419729232788, 0.6680001616477966], [-0.16238243877887726, 0.02922026254236698, 0.6680000424385071], [-0.1513497233390808, 0.02847784012556076, 0.6681945323944092], [-0.14029745757579803, 0.027922650799155235, 0.6683611273765564], [-0.12823912501335144, 0.027217961847782135, 0.6687778830528259], [-0.11711318045854568, 0.026651686057448387, 0.6681668162345886], [-0.10605540126562119, 0.026041708886623383, 0.6683611869812012], [-0.0950777679681778, 0.025547973811626434, 0.669055700302124], [-0.08298422396183014, 0.024870868772268295, 0.6687501072883606], [-0.0720130205154419, 0.024308761581778526, 0.6690556406974792], [-0.061006274074316025, 0.023660749197006226, 0.6694166660308838], [-0.05001900717616081, 0.023136282339692116, 0.6692500710487366], [-0.03800590708851814, 0.02259032428264618, 0.6696110963821411], [-0.02694779448211193, 0.0219730231910944, 0.6710556149482727], [-0.015881352126598358, 0.02150837332010269, 0.6712778806686401], [-0.00482372147962451, 0.02088848128914833, 0.6716944575309753], [0.007247734349220991, 0.020402612164616585, 0.6724722385406494], [0.01820015162229538, 0.019633209332823753, 0.672166645526886], [0.029280105605721474, 0.019039548933506012, 0.6720833778381348], [0.04031510278582573, 0.01855671964585781, 0.672888994216919], [0.052472084760665894, 0.01783231645822525, 0.6734167337417603], [0.06351131200790405, 0.017239652574062347, 0.6735556721687317], [0.07362837344408035, 0.016605908051133156, 0.6739166378974915], [0.08573660254478455, 0.01597084477543831, 0.6735000014305115], [0.09678313881158829, 0.015307310968637466, 0.6733056306838989], [0.10784538090229034, 0.014814483001828194, 0.6736111640930176], [0.11911269277334213, 0.014126699417829514, 0.6738889813423157], [0.1310858130455017, 0.013522983528673649, 0.673500120639801], [0.14229029417037964, 0.012849805876612663, 0.6736112833023071], [0.1535784751176834, 0.012304206378757954, 0.6741111874580383], [0.16490574181079865, 0.011746027506887913, 0.6741945147514343], [0.1773352324962616, 0.011176669038832188, 0.6747500896453857], [0.1885286271572113, 0.010638950392603874, 0.6745834946632385], [0.19993926584720612, 0.0100904181599617, 0.6748888492584229], [0.2111644148826599, 0.009543352760374546, 0.6747500896453857], [0.2236323356628418, 0.00893769133836031, 0.6750555634498596], [0.2350151538848877, 0.008353037759661674, 0.6751945614814758], [0.2461601346731186, 0.0077170985750854015, 0.6747778654098511], [0.257944792509079, 0.007235134020447731, 0.6756666898727417]])
pos_list_y.append([[0.03287805989384651, -0.1731848418712616, 0.676861047744751], [0.0333421491086483, -0.1643059104681015, 0.6767221689224243], [0.03388797491788864, -0.1551395058631897, 0.6759166717529297], [0.03430233895778656, -0.14595173299312592, 0.6755277514457703], [0.03489242121577263, -0.13610611855983734, 0.6760554909706116], [0.03542033210396767, -0.12697410583496094, 0.675861120223999], [0.035922057926654816, -0.1180100068449974, 0.6762500405311584], [0.036416783928871155, -0.10890789330005646, 0.6758610606193542], [0.036999356001615524, -0.09886062890291214, 0.6758055686950684], [0.037471890449523926, -0.08979037404060364, 0.6753055453300476], [0.03797590732574463, -0.08073293417692184, 0.6750555634498596], [0.03842329606413841, -0.07165365666151047, 0.6747777462005615], [0.039013639092445374, -0.06166936457157135, 0.6747223138809204], [0.039501674473285675, -0.0525621734559536, 0.6745556592941284], [0.03996606543660164, -0.043606143444776535, 0.6745555996894836], [0.04054848104715347, -0.03453114628791809, 0.6741389036178589], [0.041081469506025314, -0.024485206231474876, 0.6744167804718018], [0.04163195565342903, -0.015429877676069736, 0.6734446287155151], [0.042023058980703354, -0.006276334170252085, 0.673333466053009], [0.04260997101664543, 0.0027232361026108265, 0.6729999780654907], [0.043126869946718216, 0.012817678973078728, 0.6729723811149597], [0.04367447271943092, 0.021915733814239502, 0.6730557084083557], [0.04410352557897568, 0.030937347561120987, 0.672166645526886], [0.044659439474344254, 0.04009370878338814, 0.671972393989563], [0.045216161757707596, 0.050257608294487, 0.67208331823349], [0.04574950411915779, 0.05916349217295647, 0.6718889474868774], [0.04614277184009552, 0.06826156377792358, 0.6711111068725586], [0.04673008620738983, 0.0773124024271965, 0.6708889007568359], [0.047304365783929825, 0.08734513074159622, 0.6708056330680847], [0.047771427780389786, 0.09633158892393112, 0.6697500348091125], [0.048246052116155624, 0.10549245029687881, 0.6697501540184021], [0.048716556280851364, 0.11445926129817963, 0.6686667799949646], [0.0493135079741478, 0.12463458627462387, 0.6689723134040833], [0.049819543957710266, 0.1336548924446106, 0.6682778596878052], [0.050334345549345016, 0.14265970885753632, 0.6673334836959839], [0.050882454961538315, 0.15196441113948822, 0.6673889756202698], [0.051306843757629395, 0.1618078202009201, 0.6660279035568237], [0.05182550475001335, 0.17101329565048218, 0.6658890247344971], [0.05237006023526192, 0.18031379580497742, 0.6658055782318115], [0.05290449038147926, 0.1895407736301422, 0.6655001044273376]])

#quater 3: A = -170, -3, 175
pos_list_x.append([[-146.8629608154297, -78.87813568115234, 574.361083984375], [-137.05245971679688, -77.87235260009766, 574.3334350585938], [-127.0667495727539, -76.7088394165039, 573.4444580078125], [-117.21044158935547, -75.61255645751953, 573.3055419921875], [-107.16734313964844, -74.32677459716797, 572.3333129882812], [-97.30928802490234, -73.2387466430664, 572.444580078125], [-87.24453735351562, -71.96346282958984, 570.9444580078125], [-77.39201354980469, -70.74335479736328, 569.888916015625], [-67.51817321777344, -69.52494049072266, 569.6388549804688], [-57.604713439941406, -68.27023315429688, 568.7499389648438], [-47.6578254699707, -67.06806182861328, 567.8333740234375], [-37.69279098510742, -65.9277114868164, 567.02783203125], [-27.76127052307129, -64.5395736694336, 565.3890380859375], [-17.86269187927246, -63.481719970703125, 565.25], [-7.997305870056152, -62.239261627197266, 564.3610229492188], [1.9069218635559082, -61.14236068725586, 565.111083984375], [11.815292358398438, -59.98345947265625, 563.8887939453125], [21.746196746826172, -58.79111862182617, 563.8333129882812], [31.653478622436523, -57.6093864440918, 563.083251953125], [41.58823013305664, -56.46137619018555, 562.5276489257812], [51.53643035888672, -55.30913162231445, 562.138916015625], [61.49031448364258, -54.15339660644531, 561.22216796875], [71.45866394042969, -52.98243713378906, 560.9999389648438], [81.486328125, -51.920345306396484, 560.77783203125], [91.4029769897461, -50.61785125732422, 559.1390380859375], [101.57611846923828, -49.5760498046875, 559.3610229492188], [111.51380157470703, -48.36837387084961, 558.2222290039062], [121.59941864013672, -47.261314392089844, 557.4999389648438], [131.66012573242188, -46.002437591552734, 556.75], [141.8219451904297, -44.86293029785156, 555.6945190429688], [152.028076171875, -43.779319763183594, 555.6666870117188], [162.07762145996094, -42.422889709472656, 553.888916015625], [172.5102081298828, -41.41289520263672, 554.5277709960938], [182.42437744140625, -40.1182975769043, 552.2498779296875], [192.90814208984375, -39.17266845703125, 552.9444580078125], [202.5701904296875, -37.82404327392578, 550.166748046875], [212.81130981445312, -36.59992599487305, 550.1666870117188], [223.12957763671875, -35.39372634887695, 549.1111450195312], [232.6875, -34.13734817504883, 547.4722290039062], [243.43133544921875, -33.1346549987793, 547.9166259765625]])
pos_list_y.append([[63.66176986694336, -155.9751434326172, 558.0833129882812], [62.679595947265625, -148.16371154785156, 558.8056640625], [61.496036529541016, -138.9520721435547, 558.5000610351562], [60.538902282714844, -129.9615936279297, 559.4166870117188], [59.33983612060547, -120.84565734863281, 559.27783203125], [58.463401794433594, -112.76673126220703, 560.0000610351562], [57.34013748168945, -103.5447006225586, 559.9722290039062], [56.32613754272461, -94.52894592285156, 560.638916015625], [55.203556060791016, -85.34407043457031, 560.0001831054688], [54.344791412353516, -77.51871490478516, 562.0833129882812], [53.28556442260742, -68.48433685302734, 561.666748046875], [52.189842224121094, -59.3956184387207, 561.6389770507812], [51.10446548461914, -50.4472770690918, 562.5277099609375], [50.15449142456055, -42.460670471191406, 562.9165649414062], [49.08660888671875, -33.4140510559082, 562.611083984375], [48.05876541137695, -24.32872200012207, 562.9166870117188], [47.05195999145508, -15.375591278076172, 563.6388549804688], [46.15913009643555, -7.321791648864746, 564.3056030273438], [45.10082244873047, 1.7200961112976074, 564.4443969726562], [43.990753173828125, 10.614526748657227, 563.5000610351562], [43.092811584472656, 19.616405487060547, 566.0], [41.96016311645508, 27.559003829956055, 565.2777099609375], [40.96077346801758, 36.49176025390625, 565.2223510742188], [39.96853256225586, 45.627803802490234, 566.888916015625], [38.9556999206543, 54.62924575805664, 566.8611450195312], [38.018272399902344, 62.71944046020508, 567.638916015625], [36.98647689819336, 71.62264251708984, 568.0555419921875], [35.97026062011719, 80.58641052246094, 568.2221069335938], [34.82987594604492, 89.59269714355469, 568.138916015625], [33.91903305053711, 97.59258270263672, 568.2222290039062], [32.802223205566406, 106.44072723388672, 567.3887939453125], [31.85988998413086, 115.57645416259766, 568.5833129882812], [30.881498336791992, 124.80601501464844, 569.1945190429688], [29.851167678833008, 132.49859619140625, 567.8056030273438], [28.967031478881836, 142.04042053222656, 569.7222290039062], [27.929561614990234, 150.91482543945312, 569.75], [26.784759521484375, 159.86537170410156, 568.888916015625], [25.911413192749023, 167.93075561523438, 569.3888549804688], [24.881637573242188, 176.8872833251953, 569.1945190429688], [23.810632705688477, 186.116455078125, 569.9166870117188]])

#quater 4 A = -170, -3, -175
pos_list_x.append([[-132.91665649414062, -62.36016845703125, 568.0555419921875], [-123.07625579833984, -61.10163116455078, 568.8333740234375], [-112.94165802001953, -59.82326889038086, 569.1388549804688], [-103.22634887695312, -58.57484817504883, 570.72216796875], [-93.21966552734375, -57.322486877441406, 571.611083984375], [-83.43521881103516, -56.12222671508789, 572.97216796875], [-73.35604858398438, -54.81964111328125, 573.6944580078125], [-63.52702713012695, -53.514705657958984, 574.5000610351562], [-53.59844970703125, -52.232913970947266, 575.611083984375], [-43.68342971801758, -51.110939025878906, 576.6666870117188], [-33.8273811340332, -49.86669921875, 578.2223510742188], [-23.840328216552734, -48.661495208740234, 579.22216796875], [-14.043985366821289, -47.43338394165039, 580.5555419921875], [-4.103379249572754, -46.18020248413086, 581.9444580078125], [5.710392475128174, -45.1370849609375, 584.2222290039062], [15.704063415527344, -43.88603973388672, 585.388916015625], [25.600936889648438, -42.60779571533203, 585.9722900390625], [35.50943374633789, -41.519561767578125, 587.611083984375], [45.463111877441406, -40.27239990234375, 589.0554809570312], [55.35550308227539, -39.123844146728516, 589.75], [65.51217651367188, -37.75205993652344, 590.4166870117188], [75.40631866455078, -36.50802230834961, 591.1666259765625], [85.20445251464844, -35.27084732055664, 591.361083984375], [95.18305969238281, -33.95988082885742, 592.5833129882812], [105.10173034667969, -32.822967529296875, 592.8612060546875], [115.01346588134766, -31.417530059814453, 593.47216796875], [125.12709045410156, -30.239721298217773, 594.5833129882812], [135.41502380371094, -29.084810256958008, 596.3889770507812], [145.2690887451172, -27.781946182250977, 596.7500610351562], [155.33456420898438, -26.59187126159668, 597.4999389648438], [165.6056365966797, -25.38591766357422, 598.9444580078125], [175.65579223632812, -24.14142417907715, 599.7222290039062], [186.0113067626953, -22.957605361938477, 601.6390380859375], [196.2956085205078, -21.748449325561523, 602.7500610351562], [206.74380493164062, -20.472034454345703, 604.3889770507812], [217.02976989746094, -19.242399215698242, 605.5834350585938], [227.270263671875, -18.029518127441406, 606.72216796875], [237.4850311279297, -16.73638916015625, 607.8056030273438], [247.695068359375, -15.507787704467773, 608.638916015625], [257.5050964355469, -14.29012393951416, 608.7500610351562]])
pos_list_y.append([[78.44322967529297, -138.41334533691406, 588.5834350585938], [77.43724060058594, -130.44252014160156, 588.833251953125], [76.3375473022461, -121.37966918945312, 588.8333740234375], [75.18467712402344, -112.28331756591797, 589.2777099609375], [73.92787170410156, -103.2788314819336, 589.3056030273438], [72.91481018066406, -95.22828674316406, 589.8611450195312], [71.88240814208984, -86.31525421142578, 589.9444580078125], [70.76549530029297, -77.21339416503906, 589.75], [69.58374786376953, -68.24986267089844, 590.4443969726562], [68.53343200683594, -60.189857482910156, 590.2221069335938], [67.50325012207031, -51.11545944213867, 590.0833129882812], [66.19671630859375, -42.15148162841797, 590.5833740234375], [65.09092712402344, -33.165279388427734, 590.3887329101562], [64.10884094238281, -25.035423278808594, 590.3888549804688], [63.17118453979492, -16.16299819946289, 590.7222290039062], [61.948402404785156, -7.211845874786377, 590.2222290039062], [60.686668395996094, 1.9577627182006836, 590.111083984375], [59.7745361328125, 9.969118118286133, 590.77783203125], [58.733726501464844, 19.006349563598633, 590.4166870117188], [57.703128814697266, 27.976078033447266, 590.2498779296875], [56.534820556640625, 37.082908630371094, 591.0277099609375], [55.422019958496094, 45.03849792480469, 590.75], [54.352115631103516, 53.932533264160156, 590.3333129882812], [53.314552307128906, 62.90187072753906, 590.8888549804688], [52.1597785949707, 71.9306411743164, 590.9444580078125], [51.09523391723633, 79.97924041748047, 590.4443969726562], [50.00297546386719, 89.01118469238281, 590.6388549804688], [48.99308776855469, 98.00471496582031, 590.6666259765625], [47.81494903564453, 107.11810302734375, 591.0555419921875], [46.78644561767578, 115.12850189208984, 590.52783203125], [45.771484375, 124.17893981933594, 590.5277709960938], [44.693321228027344, 133.27911376953125, 590.9166870117188], [43.57320022583008, 142.4116973876953, 590.8055419921875], [42.568214416503906, 150.378662109375, 590.3612670898438], [41.392784118652344, 159.45953369140625, 590.3333129882812], [40.372596740722656, 168.7110137939453, 590.6112060546875], [39.23933792114258, 177.6331329345703, 590.0000610351562], [38.2999382019043, 185.80532836914062, 590.083251953125], [37.37294006347656, 195.27896118164062, 590.6388549804688]])

#quater 4 A = 170, -7, -175
pos_list_x.append([[-120.45758056640625, 21.15509605407715, 684.0834350585938], [-101.68251037597656, 16.868858337402344, 687.8890380859375], [-82.2546615600586, 12.413655281066895, 689.4722290039062], [-62.78362274169922, 7.990520477294922, 691.9166870117188], [-43.47251510620117, 3.5186681747436523, 695.1110229492188], [-23.97431182861328, -0.9660109281539917, 697.5], [-4.489467144012451, -5.425519943237305, 700.02783203125], [15.055541038513184, -9.894620895385742, 701.6110229492188], [34.648136138916016, -14.47976303100586, 704.1666259765625], [54.18240737915039, -18.860849380493164, 705.2222290039062], [73.76844787597656, -23.411184310913086, 706.0277709960938], [93.20675659179688, -27.890596389770508, 707.055419921875], [112.89408874511719, -32.32541275024414, 708.7222900390625], [132.9168701171875, -36.840824127197266, 711.9999389648438], [152.95240783691406, -41.589473724365234, 714.9722290039062], [173.0189971923828, -46.12385559082031, 716.6944580078125], [193.265869140625, -50.7901725769043, 720.0554809570312], [213.39511108398438, -55.232791900634766, 720.9443969726562], [233.59312438964844, -59.951637268066406, 722.9999389648438], [253.80465698242188, -64.6506576538086, 724.27783203125]])
pos_list_y.append([[52.1471061706543, -121.42070770263672, 695.3056640625], [55.781959533691406, -104.95073699951172, 697.8612060546875], [59.72539520263672, -87.37272644042969, 700.2222290039062], [63.322059631347656, -70.57308959960938, 701.666748046875], [67.33522033691406, -53.0428581237793, 703.3333740234375], [71.16441345214844, -36.42490005493164, 705.7778930664062], [75.0248031616211, -18.80038070678711, 706.6666870117188], [78.725830078125, -2.1185691356658936, 708.3056030273438], [82.78214263916016, 15.385778427124023, 711.8890380859375], [86.5556869506836, 32.16618728637695, 714.2778930664062], [90.76659393310547, 49.97962951660156, 718.111083984375], [94.6693115234375, 66.79291534423828, 721.0000610351562], [98.58625793457031, 84.4508056640625, 722.6944580078125], [102.39127349853516, 101.1014633178711, 724.0556640625], [106.35980224609375, 118.84471130371094, 725.6945190429688], [109.99356842041016, 135.4945831298828, 726.611328125], [113.87136840820312, 153.21527099609375, 727.4443359375], [117.72290802001953, 170.1641845703125, 729.3889770507812], [121.52191925048828, 187.71829223632812, 729.9166870117188], [125.12977600097656, 204.3221435546875, 730.4168090820312]])
def find_camera_rotation(pos_list_x, pos_list_y):

    pos_list_x = [x for x in pos_list_x if x[2] < 1000]
    pos_list_y = [y for y in pos_list_y if y[2] < 1000]

    pos_list = pos_list_x + pos_list_y
    plane_fit = fit.plane_best_fit(pos_list)
    # mat.data_show_3d(pos_list)

    #----------------Z rotation----------------
    line_fit_x = fit.line_best_fit(pos_list_x)
    plane_value = cal.get_plane_equation_from_point_normal_vector(plane_fit.vector, plane_fit.point)
    find_z_1 = (-plane_value[3] - 100*plane_value[0] - 100*plane_value[1])/plane_value[2]
    find_z_2 = (-plane_value[3] - 1*plane_value[0] - 100*plane_value[1])/plane_value[2]
    x_axis_direction_vector = np.array([100,100,find_z_1]) - np.array([1,100,find_z_2])
    rotate_angle_z = cal.get_angle_two_line_3d(line_fit_x.vector, x_axis_direction_vector)
    if line_fit_x.vector[1] < 0:
        rotate_angle_z = -rotate_angle_z
    # rotate_angle_z = -rotate_angle_z
    print('rotation Z angle = ', rotate_angle_z)

    rotz_pos_list_x = []
    rotz_pos_list_y = []
    for pos_x in pos_list_x:
        new_pos_x = pos_x*cal.Rz(math.radians(rotate_angle_z))
        rotz_pos_list_x.append(new_pos_x.tolist()[0])
    for pos_y in pos_list_y:
        new_pos_y = pos_y*cal.Rz(math.radians(rotate_angle_z))
        rotz_pos_list_y.append(new_pos_y.tolist()[0])
    # rotz_line_fit_x = fit.line_best_fit(rotz_pos_list_x)
    # mat.data_show_3d(rotz_pos_list_x ,rotz_pos_list_y)
    # mat.data_show_3d(pos_list_x)

    #----------------X rotation----------------
    rotz_line_fit_y = fit.line_best_fit(rotz_pos_list_y)
    x_axis_direction_vector_z = fit.line_best_fit(rotz_pos_list_x)
    # current_plane_rotz_equation = fit.line_best_fit(rotz_pos_list_x + rotz_pos_list_y)
    y_axis_destination_direction_vector = cal.get_line_intersection_vector_from_two_planes([1,0,0],x_axis_direction_vector_z.vector)
    # rotate_angle_zx = cal.get_angle_two_line_3d(rotz_line_fit_y.vector, 
    #                                         y_axis_destination_direction_vector)

    rotate_angle_zx = cal.get_angle_two_line_3d(rotz_line_fit_y.vector, 
                                            [0,1,0])

    # rotate_angle_zx = cal.get_andg
    # rotate_angle_zx = -180+rotate_angle_zx                                            
    # rotate_angle_zx = cal.get_angle_two_line_3d(rotz_line_fit_y.vector, [0,1,0])
    if rotz_line_fit_y.vector[0] < 0:
        if rotz_line_fit_y.vector[1] < 0 or rotz_line_fit_y.vector[2] < 0:
            rotate_angle_zx = -rotate_angle_zx
        if abs(rotate_angle_zx) > 90:
            rotate_angle_zx = -rotate_angle_zx-180
    else:
        if abs(rotate_angle_zx) > 90:
            rotate_angle_zx = 180-rotate_angle_zx
        else:
            if rotz_line_fit_y.vector[2] < 0:
                rotate_angle_zx = -rotate_angle_zx
    # if rotz_line_fit_y.vector[1]
    # rotate_angle_zx = 180 - rotate_angle_zx
    
    print('rotation X angle = ', rotate_angle_zx)

    rotzx_pos_list_x = []
    rotzx_pos_list_y = []
    for pos_x in rotz_pos_list_x:
        new_pos_x = pos_x*cal.Rx(math.radians(rotate_angle_zx))
        rotzx_pos_list_x.append(new_pos_x.tolist()[0])
    for pos_y in rotz_pos_list_y:
        new_pos_y = pos_y*cal.Rx(math.radians(rotate_angle_zx))
        rotzx_pos_list_y.append(new_pos_y.tolist()[0])
    # mat.data_show_3d(rotzx_pos_list_x  ,rotzx_pos_list_y )
    # mat.data_show_3d( rotzx_pos_list_y , rotz_pos_list_x )

    #----------------Y rotation----------------
    plane_fit = fit.plane_best_fit(rotzx_pos_list_x + rotzx_pos_list_y)
    rotate_angle_zxy = cal.get_angle_two_line_3d(plane_fit.vector, [0,0,1])
    if plane_fit.vector[0] < 0:
        rotate_angle_zxy = -rotate_angle_zxy
    # rotate_angle_zxy = -rotate_angle_zxy
    print('rotation Y angle = ',rotate_angle_zxy)

    rotzxy_pos_list_x = []
    rotzxy_pos_list_y = []
    for pos_x in rotzx_pos_list_x:
        new_pos_x = pos_x*cal.Ry(math.radians(rotate_angle_zxy))
        rotzxy_pos_list_x.append(new_pos_x.tolist()[0])
    for pos_y in rotzx_pos_list_y:
        new_pos_y = pos_y*cal.Ry(math.radians(rotate_angle_zxy))
        rotzxy_pos_list_y.append(new_pos_y.tolist()[0])

    # plane_fit = fit.plane_best_fit(rotzxy_pos_list_x + rotzxy_pos_list_y)
    # rotate_angle_zxy = cal.get_angle_two_line_3d(plane_fit.vector, [0,0,1])

    # mat.data_show_3d(rotzxy_pos_list_y + rotzxy_pos_list_x + rotzx_pos_list_x + rotzx_pos_list_y)
    mat.data_show_3d(rotzxy_pos_list_x, rotzxy_pos_list_y )
    return rotate_angle_z, rotate_angle_zx, rotate_angle_zxy
# for index in range(len(pos_list_x)):
#     pos_x = pos_list_x[index]
#     pos_y = pos_list_y[index]
#     find_camera_rotation(pos_x, pos_y)

# pos_x = pos_list_x[6]
# pos_y = pos_list_y[6]
# find_camera_rotation(pos_x, pos_y)