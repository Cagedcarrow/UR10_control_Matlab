/* -------------------------------------------------------------------
 *
 * The banner will be completed upon code publication.
 *
 * Title:   getPayloadInertiaRegressionMatrix.cpp
 * Author:  Vincenzo Petrone, Antonio Annunziata
 * Org.:    UNISA
 * Date:    July 24, 2023
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: getPayloadInertiaRegressionMatrix.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 24-Jul-2023 12:11:21
//

// Include Files
#include <ur10_inverse_dynamics_solver/getPayloadInertiaRegressionMatrix.h>
#include <cmath>

// Function Definitions
//
// getPayloadInertiaRegressionMatrix
//     PAYLOAD_INERTIA_REGRESSION_MATRIX = getPayloadInertiaRegressionMatrix(IN1,IN2)
//
// Arguments    : const double in1[6]
//                const double in2[10]
//                double payload_inertia_regression_matrix[36]
// Return Type  : void
//
namespace ur10 {
void getPayloadInertiaRegressionMatrix(const double in1[6], const double in2[10], double payload_inertia_regression_matrix[36]) {
    double b_ct_idx_239_tmp;
    double b_ct_idx_260_tmp;
    double b_t389_tmp;
    double b_t518_tmp;
    double ct_idx_100;
    double ct_idx_103;
    double ct_idx_104;
    double ct_idx_106;
    double ct_idx_116;
    double ct_idx_123;
    double ct_idx_124;
    double ct_idx_130;
    double ct_idx_131;
    double ct_idx_142;
    double ct_idx_145;
    double ct_idx_146;
    double ct_idx_147;
    double ct_idx_15;
    double ct_idx_161;
    double ct_idx_162;
    double ct_idx_162_tmp;
    double ct_idx_163;
    double ct_idx_163_tmp;
    double ct_idx_166;
    double ct_idx_167;
    double ct_idx_168;
    double ct_idx_169;
    double ct_idx_175;
    double ct_idx_176;
    double ct_idx_177;
    double ct_idx_192;
    double ct_idx_193;
    double ct_idx_198;
    double ct_idx_198_tmp;
    double ct_idx_198_tmp_tmp;
    double ct_idx_205;
    double ct_idx_211;
    double ct_idx_213;
    double ct_idx_213_tmp;
    double ct_idx_217;
    double ct_idx_217_tmp;
    double ct_idx_219;
    double ct_idx_220;
    double ct_idx_221;
    double ct_idx_226;
    double ct_idx_226_tmp;
    double ct_idx_232;
    double ct_idx_235;
    double ct_idx_235_tmp;
    double ct_idx_238;
    double ct_idx_239;
    double ct_idx_239_tmp;
    double ct_idx_242;
    double ct_idx_248;
    double ct_idx_25;
    double ct_idx_256;
    double ct_idx_258;
    double ct_idx_26;
    double ct_idx_260;
    double ct_idx_260_tmp;
    double ct_idx_261;
    double ct_idx_261_tmp;
    double ct_idx_263;
    double ct_idx_263_tmp;
    double ct_idx_264;
    double ct_idx_264_tmp;
    double ct_idx_266;
    double ct_idx_266_tmp;
    double ct_idx_267;
    double ct_idx_27;
    double ct_idx_270;
    double ct_idx_271;
    double ct_idx_272;
    double ct_idx_273;
    double ct_idx_273_tmp;
    double ct_idx_276;
    double ct_idx_278;
    double ct_idx_279;
    double ct_idx_280;
    double ct_idx_280_tmp;
    double ct_idx_282;
    double ct_idx_283;
    double ct_idx_284;
    double ct_idx_284_tmp;
    double ct_idx_291;
    double ct_idx_291_tmp;
    double ct_idx_297;
    double ct_idx_302;
    double ct_idx_303_tmp;
    double ct_idx_307;
    double ct_idx_307_tmp;
    double ct_idx_308;
    double ct_idx_308_tmp;
    double ct_idx_310;
    double ct_idx_311;
    double ct_idx_312_tmp;
    double ct_idx_313;
    double ct_idx_315;
    double ct_idx_316;
    double ct_idx_317;
    double ct_idx_320;
    double ct_idx_323;
    double ct_idx_324;
    double ct_idx_327;
    double ct_idx_329;
    double ct_idx_330;
    double ct_idx_35;
    double ct_idx_38;
    double ct_idx_49;
    double ct_idx_51;
    double ct_idx_62;
    double ct_idx_64;
    double ct_idx_65;
    double ct_idx_78;
    double ct_idx_84;
    double ct_idx_86;
    double ct_idx_87;
    double ct_idx_90;
    double ct_idx_94;
    double ct_idx_96;
    double ct_idx_96_tmp;
    double ct_idx_98;
    double ct_idx_99;
    double d;
    double d1;
    double d10;
    double d11;
    double d12;
    double d13;
    double d14;
    double d15;
    double d16;
    double d17;
    double d18;
    double d19;
    double d2;
    double d20;
    double d21;
    double d22;
    double d23;
    double d24;
    double d25;
    double d26;
    double d27;
    double d28;
    double d29;
    double d3;
    double d30;
    double d31;
    double d32;
    double d33;
    double d34;
    double d35;
    double d36;
    double d37;
    double d38;
    double d39;
    double d4;
    double d40;
    double d41;
    double d42;
    double d43;
    double d44;
    double d45;
    double d46;
    double d47;
    double d48;
    double d49;
    double d5;
    double d50;
    double d51;
    double d52;
    double d53;
    double d54;
    double d55;
    double d56;
    double d57;
    double d6;
    double d7;
    double d8;
    double d9;
    double t10;
    double t103;
    double t105;
    double t107;
    double t108;
    double t109;
    double t11;
    double t12;
    double t124;
    double t125;
    double t126;
    double t127;
    double t128;
    double t129;
    double t13;
    double t132;
    double t14;
    double t140;
    double t141;
    double t143;
    double t144;
    double t145;
    double t148;
    double t149;
    double t15;
    double t152;
    double t153;
    double t156;
    double t157;
    double t160;
    double t161;
    double t165;
    double t167;
    double t168;
    double t169;
    double t170;
    double t173;
    double t174;
    double t175;
    double t176;
    double t177;
    double t178;
    double t185;
    double t188;
    double t189;
    double t191;
    double t193;
    double t194;
    double t195;
    double t196;
    double t199;
    double t2;
    double t21;
    double t213;
    double t22;
    double t228;
    double t23;
    double t237;
    double t240;
    double t247;
    double t26;
    double t268;
    double t27;
    double t28;
    double t29;
    double t293;
    double t294;
    double t3;
    double t31;
    double t313;
    double t314;
    double t32;
    double t33;
    double t34;
    double t35;
    double t36;
    double t363;
    double t365;
    double t37;
    double t38;
    double t380;
    double t381;
    double t382;
    double t383;
    double t384;
    double t385;
    double t386;
    double t387;
    double t389;
    double t389_tmp;
    double t391;
    double t4;
    double t40;
    double t430;
    double t431;
    double t437;
    double t438;
    double t440;
    double t440_tmp;
    double t449;
    double t450;
    double t477;
    double t479;
    double t48;
    double t480;
    double t497;
    double t498;
    double t499;
    double t5;
    double t50;
    double t500;
    double t51;
    double t518;
    double t518_tmp;
    double t52;
    double t54;
    double t543;
    double t543_tmp;
    double t57;
    double t58;
    double t59;
    double t599;
    double t60;
    double t62;
    double t62_tmp;
    double t64;
    double t64_tmp;
    double t65;
    double t65_tmp;
    double t68;
    double t69;
    double t696;
    double t7;
    double t71;
    double t72;
    double t76;
    double t77;
    double t79;
    double t8;
    double t80;
    double t81;
    double t83;
    double t84;
    double t85;
    double t88;
    double t89;
    double t9;
    double t90;
    double t92;
    double t93;
    double t95;
    double t96;
    double t99;
    //     This function was generated by the Symbolic Math Toolbox version 9.3.
    //     24-Jul-2023 12:04:33
    t2 = std::cos(in1[2]);
    t3 = std::cos(in1[3]);
    t4 = std::cos(in1[4]);
    t5 = std::cos(in1[5]);
    t7 = std::sin(in1[2]);
    t8 = std::sin(in1[3]);
    t9 = std::sin(in1[4]);
    t10 = std::sin(in1[5]);
    t11 = in1[1] + in1[2];
    t12 = in1[1] + in1[4];
    t13 = in1[3] + in1[4];
    t14 = in1[3] + in1[5];
    t15 = in1[4] + in1[5];
    t21 = in1[3] * 2.0;
    t22 = in1[4] * 2.0;
    t23 = in1[5] * 2.0;
    t26 = t4 * t4;
    t27 = t5 * t5;
    t28 = std::sin(t23);
    t29 = in2[7] * t5;
    t31 = in2[4] * t9;
    t32 = in2[5] * t10;
    t33 = in1[3] + t11;
    t34 = in1[4] + t11;
    t35 = in1[2] + t13;
    t36 = in1[5] + t12;
    t37 = in1[2] + t14;
    t38 = in1[5] + t13;
    t40 = std::sin(t12);
    t50 = t11 + t13;
    t51 = t11 + t14;
    t52 = t11 + t15;
    t57 = in2[9] * (in2[0] * in2[0]);
    t58 = in2[9] * (in2[1] * in2[1]);
    t59 = in2[9] * (in2[2] * in2[2]);
    t60 = in1[4] + t15;
    t62_tmp = in2[1] * in2[2] * in2[9];
    t62 = t62_tmp * t5;
    t64_tmp = in2[0] * in2[1] * in2[9];
    t64 = t64_tmp * t9;
    t65_tmp = in2[0] * in2[2] * in2[9];
    t65 = t65_tmp * t10;
    t68 = in2[5] * t5 * t9;
    t69 = in2[7] * t9 * t10;
    t79 = in1[1] - in1[4];
    t80 = in1[3] - in1[4];
    t81 = in1[3] - in1[5];
    t88 = t15 * 2.0;
    t90 = t11 - in1[4];
    t92 = t12 - in1[5];
    t95 = t15 - in1[1];
    t96 = t13 - in1[5];
    t108 = t22 - in1[5];
    t143 = t65_tmp * t5 * t9;
    t149 = t62_tmp * t9 * t10;
    t199 = t11 * 2.0 + t21;
    t228 = t11 - t15;
    t389_tmp = in2[0] * in2[9];
    t599 = t389_tmp * t3;
    b_t389_tmp = t599 * t5;
    t389 = b_t389_tmp * 0.5716;
    t518_tmp = t389_tmp * t2;
    b_t518_tmp = t518_tmp * t3;
    t518 = b_t518_tmp * t5 * 0.6127;
    t543_tmp = in2[1] * in2[9];
    t543 = t543_tmp * t7 * t8 * t10 * 0.6127;
    t48 = std::cos(t33);
    t54 = std::sin(t34);
    t71 = std::cos(t50);
    t72 = std::cos(t51);
    t76 = t11 + t38;
    t77 = std::sin(t51);
    t83 = in2[3] * t27;
    t84 = in2[6] * t26;
    t85 = in2[6] * t27;
    t93 = in1[5] + t79;
    t99 = std::sin(t79);
    t103 = t13 + t34;
    t105 = in1[5] + t51;
    t109 = t22 - t23;
    t124 = t33 - in1[4];
    t125 = t33 - t22;
    t126 = t33 - in1[5];
    t127 = t33 - t23;
    t128 = t34 - in1[5];
    t129 = in1[5] + t90;
    t132 = std::sin(t90);
    t144 = t23 + t50;
    t145 = t15 + t50;
    t160 = t27 * t57;
    t161 = t26 * t58;
    t165 = t11 + t50;
    t173 = -in1[5] + t50;
    t174 = -in1[4] + t51;
    t175 = -t23 + t50;
    t176 = -t22 + t51;
    t185 = t38 + t52;
    t213 = t34 + t96;
    t237 = -t15 + t33;
    t240 = t33 - t88;
    t247 = t50 + t51;
    t268 = t51 * 2.0;
    t313 = -in1[5] + t199;
    t314 = -t23 + t199;
    t440_tmp = in2[2] * in2[9];
    t440 = t440_tmp * t40 * 0.30635;
    t477 = -t15 + t199;
    t479 = (t21 + t90) + t228;
    t480 = -t88 + t199;
    t696 = in2[9] * t40 * 0.02824547;
    t89 = std::sin(t76);
    t107 = std::cos(t76);
    t140 = std::cos(t103);
    t141 = std::cos(t105);
    t148 = std::sin(t105);
    t152 = std::cos(t144);
    t153 = std::cos(t145);
    t156 = std::sin(t144);
    t157 = std::sin(t145);
    t167 = std::cos(t124);
    t168 = std::cos(t125);
    t169 = std::cos(t126);
    t170 = std::cos(t127);
    t177 = std::sin(t126);
    t178 = std::sin(t127);
    t188 = std::cos(t173);
    t189 = std::cos(t174);
    t191 = std::cos(t176);
    t193 = std::sin(t173);
    t194 = std::sin(t174);
    t195 = std::sin(t175);
    t196 = std::sin(t176);
    t34 = -in1[4] + t105;
    t40 = -t22 + t105;
    t145 = -t23 + t103;
    t105 = -t15 + t126;
    t103 = -t15 + t124;
    t293 = t51 + t76;
    t294 = t50 + t76;
    d = std::sin(t80);
    d1 = std::cos(t88);
    d2 = std::sin(t88);
    d3 = std::sin(in1[2] + t80);
    d4 = std::sin(t12 + t33);
    d5 = std::cos(t108);
    d6 = std::cos(t109);
    d7 = std::sin(t108);
    d8 = std::sin(t109);
    d9 = std::sin(t165);
    d10 = std::cos(t175);
    d11 = std::cos(t199);
    d12 = std::sin(t33 + t79);
    d13 = std::cos(t185);
    d14 = std::sin(t185);
    d15 = std::cos(t22);
    d16 = std::cos(t33 + t50);
    d17 = std::cos(t34);
    d18 = std::cos(t213);
    d19 = std::cos(t23);
    d20 = std::cos(t40);
    d21 = std::cos(t145);
    d22 = std::sin(t34);
    d23 = std::sin(t213);
    d24 = std::sin(t40);
    d25 = std::sin(t145);
    d26 = std::cos(t247);
    d27 = std::sin(t247);
    d28 = std::sin(t33 + t90);
    d29 = std::cos(t237);
    d30 = std::cos(t105);
    d31 = std::cos(t103);
    d32 = std::cos(t240);
    d33 = std::sin(t237);
    d34 = std::sin(t105);
    d35 = std::sin(t103);
    d36 = std::sin(t240);
    d37 = std::cos(t50 * 2.0);
    d38 = std::cos(t268);
    d39 = std::sin(t268);
    d40 = std::cos((t11 + t21) + t90);
    d41 = std::cos(-t22 + t199);
    d42 = std::cos(t314);
    d43 = std::sin(t314);
    d44 = std::cos(t293);
    d45 = std::cos(t294);
    d46 = std::sin(t293);
    d47 = std::sin(t294);
    d48 = std::sin(t13);
    d49 = std::sin(t35);
    d50 = std::cos(t477);
    d51 = std::cos(t479);
    d52 = std::cos(t480);
    d53 = std::sin(t477);
    d54 = std::sin(t479);
    d55 = std::sin(t480);
    d56 = std::cos(t60);
    d57 = std::sin(t60);
    ct_idx_15 = in1[1] + t51;
    ct_idx_25 = t64_tmp * t28;
    ct_idx_26 = t35 - in1[5];
    ct_idx_27 = t37 - in1[4];
    ct_idx_35 = t12 + t51;
    ct_idx_38 = t4 * t68 * 2.0;
    ct_idx_49 = t27 * t58;
    ct_idx_51 = t11 + t51;
    ct_idx_62 = t27 * t84;
    ct_idx_64 = -(t4 * t69 * 2.0);
    ct_idx_65 = t11 + t76;
    ct_idx_78 = in1[1] + t126;
    ct_idx_84 = t33 + t51;
    ct_idx_86 = t33 + t92;
    ct_idx_87 = t51 + t79;
    ct_idx_90 = -(t26 * t59);
    ct_idx_94 = t4 * t143 * 2.0;
    ct_idx_96_tmp = in1[2] + in1[3];
    ct_idx_96 = ct_idx_96_tmp - t15;
    ct_idx_98 = t440_tmp * 0.1844;
    ct_idx_99 = -(t26 * t83);
    ct_idx_100 = t11 + t126;
    ct_idx_103 = -(in2[4] * t5 * t10 * t26 * 2.0);
    ct_idx_104 = in2[7] * t107 / 4.0;
    ct_idx_106 = in2[5] * t89 / 4.0;
    ct_idx_116 = -(t4 * t149 * 2.0);
    ct_idx_123 = -in1[5] + t165;
    ct_idx_124 = t51 + t90;
    ct_idx_130 = t26 * t160;
    ct_idx_131 = t27 * t161;
    ct_idx_142 = in2[4] * t156 / 4.0;
    ct_idx_145 = -(t64_tmp * t5 * t10 * t26 * 2.0);
    ct_idx_146 = t62_tmp * t107 / 4.0;
    ct_idx_147 = t65_tmp * t89 / 4.0;
    ct_idx_161 = in1[1] + t237;
    ct_idx_162_tmp = t543_tmp * t5;
    ct_idx_162 = ct_idx_162_tmp * 0.2314;
    ct_idx_163_tmp = t389_tmp * t10;
    ct_idx_163 = ct_idx_163_tmp * 0.2314;
    ct_idx_166 = in2[3] * t152 / 8.0;
    ct_idx_167 = in2[6] * t152 / 8.0;
    ct_idx_168 = in2[7] * t188 / 4.0;
    ct_idx_169 = in2[7] * t189 / 4.0;
    ct_idx_175 = in2[4] * t195 / 4.0;
    ct_idx_176 = in2[5] * t193 / 4.0;
    ct_idx_177 = in2[5] * t194 / 4.0;
    t145 = t50 + t126;
    t105 = t51 + t124;
    t103 = t50 + t127;
    t88 = t51 + t125;
    ct_idx_192 = t33 + t228;
    ct_idx_193 = t440_tmp * t26 * -0.1844;
    t80 = t50 + t144;
    ct_idx_198_tmp_tmp = t440_tmp * t3;
    ct_idx_198_tmp = ct_idx_198_tmp_tmp * t9;
    ct_idx_198 = ct_idx_198_tmp * 1.1432;
    ct_idx_205 = t440_tmp * t54 * 0.2858;
    t185 = t51 + t174;
    t23 = t50 + t173;
    t109 = t51 + t176;
    t108 = t50 + t175;
    t21 = t389_tmp * t4;
    t40 = t21 * t5;
    ct_idx_211 = t40 * t9 * 0.1844;
    ct_idx_213_tmp = ct_idx_162_tmp * t8;
    ct_idx_213 = -(ct_idx_213_tmp * 1.1432);
    ct_idx_217_tmp = t389_tmp * t8 * t10;
    ct_idx_217 = -(ct_idx_217_tmp * 1.1432);
    ct_idx_219 = t389_tmp * std::cos(t36) * 0.153175;
    ct_idx_220 = t543_tmp * std::sin(t36) * 0.153175;
    t13 = t599 * t4;
    ct_idx_221 = t13 * t5 * 1.1432;
    t213 = -t15 + t313;
    ct_idx_226_tmp = t543_tmp * t4;
    ct_idx_226 = -(ct_idx_226_tmp * t9 * t10 * 0.1844);
    ct_idx_232 = t389_tmp * std::cos(t52) * 0.1429;
    ct_idx_235_tmp = t543_tmp * t3;
    t477 = ct_idx_235_tmp * t4;
    t479 = t477 * t10;
    ct_idx_235 = -(t479 * 1.1432);
    ct_idx_238 = in1[5] + t35;
    ct_idx_239_tmp = t440_tmp * t2;
    b_ct_idx_239_tmp = ct_idx_239_tmp * t3 * t9;
    ct_idx_239 = b_ct_idx_239_tmp * 0.6127;
    ct_idx_242 = t543_tmp * std::sin(t52) * 0.1429;
    ct_idx_248 = in2[9] * 0.02188733;
    ct_idx_256 = t440_tmp * t99 * 0.30635;
    ct_idx_258 = t440_tmp * t132 * 0.2858;
    ct_idx_260_tmp = t543_tmp * t2;
    b_ct_idx_260_tmp = ct_idx_260_tmp * t5 * t8;
    ct_idx_260 = -(b_ct_idx_260_tmp * 0.6127);
    ct_idx_261_tmp = ct_idx_235_tmp * t5 * t7;
    ct_idx_261 = -(ct_idx_261_tmp * 0.6127);
    ct_idx_263_tmp = t518_tmp * t8 * t10;
    ct_idx_263 = -(ct_idx_263_tmp * 0.6127);
    ct_idx_264_tmp = t599 * t7 * t10;
    ct_idx_264 = -(ct_idx_264_tmp * 0.6127);
    ct_idx_266_tmp = t440_tmp * t7 * t8 * t9;
    ct_idx_266 = -(ct_idx_266_tmp * 0.6127);
    ct_idx_267 = in2[9] * 0.34861389;
    ct_idx_270 = t4 * t518;
    ct_idx_271 = t389_tmp * std::cos(t92) * 0.153175;
    ct_idx_272 = t389_tmp * std::cos(t95) * 0.153175;
    ct_idx_273_tmp = in2[9] * t3;
    t294 = ct_idx_273_tmp * t9;
    ct_idx_273 = t294 * 0.10540304;
    ct_idx_276 = t4 * t543;
    ct_idx_278 = t543_tmp * std::sin(t92) * 0.153175;
    ct_idx_279 = t543_tmp * std::sin(t95) * 0.153175;
    ct_idx_280_tmp = in2[9] * t2;
    ct_idx_280 = ct_idx_280_tmp * 0.35021932;
    t34 = in2[9] * t8;
    ct_idx_282 = -(t34 * 0.13226824);
    ct_idx_283 = -(t34 * 0.06613412);
    t199 = ct_idx_260_tmp * t3;
    ct_idx_284_tmp = t199 * t4 * t10;
    ct_idx_284 = ct_idx_284_tmp * -0.6127;
    ct_idx_291_tmp = t40 * t7 * t8;
    ct_idx_291 = ct_idx_291_tmp * -0.6127;
    ct_idx_297 = -(in2[9] * t26 * 0.00850084);
    ct_idx_302 = in2[9] * t54 * 0.02635076;
    ct_idx_303_tmp = ct_idx_280_tmp * t3 * t9;
    t314 = ct_idx_303_tmp * 0.05649094;
    ct_idx_307_tmp = ct_idx_280_tmp * t8;
    ct_idx_307 = -(ct_idx_307_tmp * 0.07088939);
    ct_idx_308_tmp = ct_idx_273_tmp * t7;
    ct_idx_308 = -(ct_idx_308_tmp * 0.07088939);
    ct_idx_310 = t389_tmp * std::cos(t228) * 0.1429;
    ct_idx_311 = t543_tmp * std::sin(t228) * 0.1429;
    ct_idx_312_tmp = in2[9] * t7 * t8 * t9;
    t293 = -(ct_idx_312_tmp * 0.05649094);
    ct_idx_313 = in2[9] * t132 * 0.02635076;
    ct_idx_315 = in2[9] * t99 * 0.02824547;
    ct_idx_316 = ((((-t29 - t32) - t62) - t65) - ct_idx_162_tmp * 0.0922) - ct_idx_163_tmp * 0.0922;
    t40 = ct_idx_226_tmp * t5;
    t247 = t21 * t10;
    t22 = t389_tmp * t5;
    t240 = t22 * t9;
    t268 = t543_tmp * t9 * t10;
    ct_idx_317 =
        (((((((((in2[8] * t4 + t68) + t4 * t57) + t4 * t58) - t69) + t143) - t149) + t240 * 0.0922) - t268 * 0.0922) + t40 * 0.1157) + t247 * 0.1157;
    ct_idx_320 = in1[4] - in1[5];
    ct_idx_323 = in2[8] * t26;
    ct_idx_324 = in2[4] * t28;
    ct_idx_327 = in1[2] + t81;
    ct_idx_329 = t14 - in1[4];
    ct_idx_330 = t15 - in1[3];
    t363 = in2[3] * d10 / 8.0;
    t365 = in2[6] * d10 / 8.0;
    t380 = std::cos(t145);
    t381 = std::cos(t105);
    t382 = std::cos(t103);
    t383 = std::cos(t88);
    t384 = std::sin(t145);
    t385 = std::sin(t105);
    t386 = std::sin(t103);
    t387 = std::sin(t88);
    t391 = t64_tmp * t156 / 4.0;
    t430 = t62_tmp * t188 / 4.0;
    t431 = t62_tmp * t189 / 4.0;
    t195 = t64_tmp * t195 / 4.0;
    t437 = t65_tmp * t193 / 4.0;
    t438 = t65_tmp * t194 / 4.0;
    t449 = std::cos(t80);
    t450 = std::sin(t80);
    t149 = in2[7] * d29 / 4.0;
    t26 = in2[5] * d33 / 4.0;
    t28 = t152 * t57 / 8.0;
    t69 = t152 * t58 / 8.0;
    t497 = std::cos(t185);
    t498 = std::cos(t23);
    t499 = std::cos(t109);
    t500 = std::cos(t108);
    t152 = std::sin(t185);
    t156 = std::sin(t23);
    t143 = std::sin(t109);
    t68 = std::sin(t108);
    t3 = d10 * t57 / 8.0;
    t2 = d10 * t58 / 8.0;
    t99 = std::cos(t213);
    t132 = std::sin(t213);
    t52 = t62_tmp * d29 / 4.0;
    t599 = t65_tmp * d33 / 4.0;
    t228 = t389_tmp * std::cos(t93) * 0.153175;
    t54 = t543_tmp * std::sin(t93) * 0.153175;
    t92 = t389_tmp * std::cos(t128) * 0.1429;
    t95 = t389_tmp * std::cos(t129) * 0.1429;
    t50 = t543_tmp * std::sin(t128) * 0.1429;
    t176 = t543_tmp * std::sin(t129) * 0.1429;
    t175 = (((t389 - ct_idx_235_tmp * t10 * 0.5716) + t40 * t8 * -0.5716) + t21 * t8 * t10 * -0.5716) + ct_idx_317;
    t34 = t5 * t9 * t10;
    t36 = t440_tmp * t4;
    t35 = in2[9] * t4;
    t51 = ((((((((((((((((-t64 - t31) + in2[6] * t5 * t9 * t10) - t4 * t29) - t4 * t32) + t27 * t31 * 2.0) - in2[3] * t5 * t9 * t10) + t4 * -t62) +
                   t4 * -t65) +
                  t27 * t64 * 2.0) +
                 t34 * t57) -
                t34 * t58) +
               t40 * -0.0922) +
              t247 * -0.0922) -
             t36 * 0.1157) +
            t240 * 0.1157) -
           t268 * 0.1157) -
          t35 * 0.01066754;
    t124 = in2[4] * d22 / 4.0;
    t127 = in2[3] * d17 / 8.0;
    t125 = in2[6] * d17 / 8.0;
    t144 = in2[4] * d34 / 4.0;
    t174 = in2[3] * d30 / 8.0;
    t173 = in2[6] * d30 / 8.0;
    t237 = t64_tmp * d22 / 4.0;
    t165 = t64_tmp * d34 / 4.0;
    t90 = d17 * t57 / 8.0;
    t76 = d17 * t58 / 8.0;
    t79 = d30 * t57 / 8.0;
    t12 = d30 * t58 / 8.0;
    t126 = (((t36 * t8 * 0.5716 + t543_tmp * t8 * t9 * t10 * 0.5716) - t22 * t8 * t9 * 0.5716) + t35 * t8 * 0.05270152) + t51;
    t480 =
        (((((((t518 + t543) - t199 * t10 * 0.6127) - t22 * t7 * t8 * 0.6127) + ct_idx_260_tmp * t4 * t5 * t8 * -0.6127) + t477 * t5 * t7 * -0.6127) +
          t518_tmp * t4 * t8 * t10 * -0.6127) +
         t13 * t7 * t10 * -0.6127) +
        t175;
    t60 = (((((((((((((((((((((((((in2[6] + t57) + t59) + t83) + ct_idx_323) + ct_idx_324) + ct_idx_25) - t84) - t85) + ct_idx_38) + t161) +
                         ct_idx_49) +
                        ct_idx_62) +
                       ct_idx_64) -
                      t160) +
                     ct_idx_90) +
                    ct_idx_94) +
                   ct_idx_98) +
                  ct_idx_99) +
                 ct_idx_103) +
                ct_idx_116) +
               ct_idx_130) +
              ct_idx_145) +
             ct_idx_162) +
            ct_idx_163) -
           ct_idx_131) +
          ct_idx_193;
    t13 = ((((((((((t60 + ct_idx_198_tmp * 0.5716) + ct_idx_211) - ct_idx_213_tmp * 0.5716) - ct_idx_217_tmp * 0.5716) + t4 * t389) + ct_idx_226) +
              t479 * -0.5716) +
             ct_idx_248) +
            ct_idx_283) +
           t294 * 0.05270152) +
          ct_idx_297;
    t477 = ((((((t60 + ct_idx_198) + ct_idx_211) + ct_idx_213) + ct_idx_217) + ct_idx_221) + ct_idx_226) + ct_idx_235;
    t479 = ((((((((((((((((((t477 + ct_idx_239) + ct_idx_260) + ct_idx_261) + ct_idx_263) + ct_idx_264) + ct_idx_266) + ct_idx_267) + ct_idx_270) +
                     ct_idx_273) +
                    ct_idx_276) +
                   ct_idx_280) +
                  ct_idx_282) +
                 ct_idx_284) +
                ct_idx_291) +
               ct_idx_297) +
              t314) +
             ct_idx_307) +
            ct_idx_308) +
           t293;
    t294 = (((((((ct_idx_239_tmp * t4 * t8 * 0.6127 + ct_idx_198_tmp_tmp * t4 * t7 * 0.6127) + ct_idx_260_tmp * t8 * t9 * t10 * 0.6127) +
                ct_idx_235_tmp * t7 * t9 * t10 * 0.6127) -
               t518_tmp * t5 * t8 * t9 * 0.6127) -
              b_t389_tmp * t7 * t9 * 0.6127) +
             ct_idx_280_tmp * t4 * t8 * 0.05649094) +
            ct_idx_273_tmp * t4 * t7 * 0.05649094) +
           t126;
    t314 = (((((((((((((ct_idx_239 + ct_idx_260) + ct_idx_261) + ct_idx_263) + ct_idx_264) + ct_idx_266) + ct_idx_270) + ct_idx_276) + ct_idx_284) +
                ct_idx_291) +
               t314) +
              ct_idx_307) +
             ct_idx_308) +
            t293) +
           t13;
    t293 = t543_tmp * t189;
    t199 = t389_tmp * t194;
    t22 = t543_tmp * t107;
    t21 = t389_tmp * t89;
    t268 = t389_tmp * t193;
    t240 = t543_tmp * t188;
    t247 = t543_tmp * d29;
    t108 = t389_tmp * d33;
    t109 = t543_tmp * t72;
    t185 = t389_tmp * t77;
    t23 = t543_tmp * t169;
    t213 = t389_tmp * t177;
    t80 = ((((((((((((((((((((((((((((((((((((((((((((((((((((((((in2[7] * t72 / 2.0 + in2[8] * t71 / 2.0) + in2[5] * t77 / 2.0) + ct_idx_104) + ct_idx_106) + t62_tmp * t72 / 2.0) + t65_tmp * t77 / 2.0) + ct_idx_146) + ct_idx_147) + in2[7] * t169 / 2.0) + t57 * t71 / 2.0) + t58 * t71 / 2.0) + ct_idx_169) + ct_idx_176) + ct_idx_177) - in2[8] * t167 / 2.0) - in2[5] * t177 / 2.0) - ct_idx_168) + t62_tmp * t169 / 2.0) - t65_tmp * t177 / 2.0) + t431) + t437) + t438) + ct_idx_220) - t167 * t57 / 2.0) - t167 * t58 / 2.0) + t26) - t430) - ct_idx_219) - t149) + t109 * 0.12805) + ct_idx_242) + t185 * 0.12805) - ct_idx_232) + t22 * 0.09295) + t21 * 0.09295) + t599) + ct_idx_271) + t228) + ct_idx_278) + ct_idx_279) - t52) + t293 * 0.0351) + t199 * 0.0351) - ct_idx_272) - t54) + t92) + t95) + t50) + t213 * 0.03585) + t240 * 0.04685) + t108 * 0.011) - t23 * 0.03585) - t176) - t268 * 0.04685) - t247 * 0.011) - ct_idx_310) - ct_idx_311;
    t103 = t440_tmp * t71;
    t88 = t440_tmp * t167;
    t40 = in2[9] * t71;
    t145 = in2[9] * t48;
    t34 = in2[9] * t167;
    t105 = (((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((-(in2[3] * t48 / 2.0) - in2[6] * t48 / 2.0) - t48 * t59) + in2[3] * t141 / 4.0) + in2[4] * t148 / 2.0) - ct_idx_104) - ct_idx_106) + ct_idx_142) - t48 * t57 / 2.0) - t48 * t58 / 2.0) + in2[3] * t170 / 4.0) - in2[6] * t141 / 4.0) + ct_idx_166) + ct_idx_169) + ct_idx_175) + ct_idx_176) + ct_idx_177) + t64_tmp * t148 / 2.0) - ct_idx_146) - ct_idx_147) - in2[6] * t170 / 4.0) - in2[4] * t178 / 2.0) + t365) - ct_idx_167) - ct_idx_168) + t141 * t58 / 4.0) + t391) - t363) + t440_tmp * t48 * -0.1844) - t64_tmp * t178 / 2.0) - t141 * t57 / 4.0) + t170 * t58 / 4.0) + t124) + t431) + t195) + t437) + t438) - ct_idx_205) + ct_idx_220) - t170 * t57 / 4.0) + t127) + t149) + t144) - t430) - t440) + t69) - ct_idx_219) + t173) - t125) - t26) - t103 * 0.1398) + ct_idx_242) + t3) - t28) + t237) + ct_idx_256) + t293 * 0.011) + t199 * 0.011) - t174) + ct_idx_258) - ct_idx_232) - t2) + t22 * 0.04685) + t52) + t21 * 0.04685) + t165) + t76) + t54) + ct_idx_279) - t599) + t79) - t90) - ct_idx_271) - t228) - ct_idx_272) - ct_idx_278) - t88 * 0.0241) + t176) - t12) + t268 * 0.09295) - t145 * 0.00850084) - t92) - t95) - t50) - t40 * 0.01288956) - t240 * 0.09295) - t696) - ct_idx_302) + t247 * 0.0351) + ct_idx_313) - ct_idx_310) - ct_idx_311) - t108 * 0.0351) + ct_idx_315) - t34 * 0.00222202;
    t145 = (((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((ct_idx_104 + ct_idx_106) + in2[8] * t140 / 4.0) + in2[7] * t153 / 4.0) + in2[5] * t157 / 4.0) + ct_idx_146) + ct_idx_147) + ct_idx_167) + ct_idx_168) + ct_idx_169) + in2[7] * t191 / 4.0) - ct_idx_142) + ct_idx_177) + in2[5] * t196 / 4.0) + in2[3] * t168 / 8.0) - in2[3] * t140 / 8.0) + in2[6] * t168 / 8.0) - in2[6] * t140 / 8.0) - in2[8] * t168 / 4.0) + t363) - ct_idx_166) - ct_idx_175) - ct_idx_176) + t62_tmp * t153 / 4.0) + t65_tmp * t157 / 4.0) - t365) + t168 * t59 / 4.0) - t140 * t59 / 4.0) + t140 * t57 / 8.0) + t140 * t58 / 8.0) + in2[6] * d13 / 16.0) + t124) + in2[5] * d23 / 4.0) + t430) + t431) + t62_tmp * t191 / 4.0) - t391) + t438) + t65_tmp * t196 / 4.0) + t127) + in2[3] * d20 / 16.0) - in2[3] * d13 / 16.0) + in2[6] * d21 / 16.0) + t149) - in2[7] * d18 / 4.0) + t144) + in2[4] * d24 / 8.0) + in2[4] * d25 / 8.0) - in2[4] * d14 / 8.0) + in2[5] * d35 / 4.0) - t195) - t437) + t28) - t168 * t57 / 8.0) - t168 * t58 / 8.0) + in2[3] * d32 / 16.0) - in2[3] * d21 / 16.0) + t173) - t125) - in2[6] * d20 / 16.0) - in2[7] * d31 / 4.0) - t26) - t103 * 0.0241) + t109 * 0.08195) + t185 * 0.08195) + t2) - t69) + t22 * 0.0351) + t21 * 0.0351) + t237) + t65_tmp * d23 / 4.0) + d13 * t57 / 16.0) + t240 * 0.011) - t174) - in2[6] * d32 / 16.0) - in2[4] * d36 / 8.0) - t3) + t52) - t62_tmp * d18 / 4.0) + t165) + t65_tmp * d35 / 4.0) + t64_tmp * d24 / 8.0) + t64_tmp * d25 / 8.0) - t64_tmp * d14 / 8.0) + t76) + d21 * t57 / 16.0) + d20 * t58 / 16.0) - d13 * t58 / 16.0) + t543_tmp * t153 * 0.02305) - t268 * 0.011) + t389_tmp * t157 * 0.02305) + t88 * 0.1398) + t440_tmp * t168 * 0.0461) - t440_tmp * t140 * 0.0461) - t62_tmp * d31 / 4.0) - t599) + t79) - t90) - d20 * t57 / 16.0) + d32 * t58 / 16.0) - d21 * t58 / 16.0) + t543_tmp * t191 * 0.02305) + t389_tmp * t196 * 0.02305) + t23 * 0.08195) - t64_tmp * d36 / 8.0) - t12) - d32 * t57 / 16.0) + t293 * 0.09295) + t199 * 0.09295) - t213 * 0.08195) - t40 * 0.00222202) + t389_tmp * d23 * 0.02305) + t145 * 0.01896323) - t543_tmp * d18 * 0.02305) + t389_tmp * d35 * 0.02305) + t34 * 0.01288956) - t543_tmp * d31 * 0.02305) + t108 * 0.04685) + in2[9] * t168 * 0.00212521) - in2[9] * t140 * 0.00212521) - t247 * 0.04685;
    t40 =
        ((((((((((((ct_idx_205 + ct_idx_232) + ct_idx_258) - ct_idx_242) + in2[9] * std::sin(t11) * 0.09368524) + t92) + t50) + t176) + ct_idx_302) -
            t95) +
           ct_idx_313) -
          ct_idx_310) -
         ct_idx_311) +
        t145;
    t34 = ((((((((((((t440 + ct_idx_219) - ct_idx_220) + ct_idx_256) + ct_idx_271) + ct_idx_278) + t54) + ct_idx_279) - t228) - ct_idx_272) +
             in2[9] * std::sin(in1[1]) * 0.10042153) +
            t696) +
           ct_idx_315) +
          t40;
    payload_inertia_regression_matrix[0] =
        (((((((((((((((((((((((((((((((((((((((((in2[3] * 0.375 + in2[6] * 0.375) + in2[8] / 4.0) + in2[9] * 0.39099601) + t57 * 0.625) +
                                             t58 * 0.625) +
                                            t59 * 0.75) -
                                           ct_idx_324 / 4.0) -
                                          ct_idx_25 / 4.0) +
                                         ct_idx_280) +
                                        ct_idx_283) +
                                       in2[3] * d15 / 8.0) -
                                      in2[3] * d19 / 8.0) +
                                     in2[3] * d1 / 16.0) +
                                    in2[3] * d6 / 16.0) +
                                   in2[3] * d11 / 8.0) -
                                  in2[3] * d37 / 16.0) -
                                 in2[3] * d38 * 0.1875) -
                                in2[3] * d41 / 16.0) -
                               in2[3] * d42 * 0.1875) -
                              in2[3] * d44 / 8.0) +
                             in2[4] * d2 / 8.0) +
                            in2[3] * t382 / 8.0) -
                           in2[4] * d8 / 8.0) -
                          in2[3] * t449 / 32.0) -
                         in2[3] * t497 / 8.0) -
                        in2[3] * t499 / 32.0) -
                       in2[3] * t500 / 32.0) +
                      ((((((((((((((((((((((((((in2[3] * t99 / 8.0 - in2[4] * d39 * 0.375) - in2[3] * d52 / 32.0) - in2[5] * d57 / 4.0) +
                                             in2[4] * d43 * 0.375) -
                                            in2[4] * d46 / 4.0) -
                                           in2[4] * t386 / 4.0) -
                                          in2[5] * d7 / 4.0) -
                                         in2[4] * t450 / 16.0) -
                                        in2[4] * t152 / 4.0) -
                                       in2[4] * t143 / 16.0) +
                                      in2[4] * t68 / 16.0) +
                                     in2[5] * d27 / 4.0) -
                                    in2[4] * t132 / 4.0) +
                                   in2[4] * d55 / 16.0) +
                                  in2[5] * d47 / 8.0) -
                                 in2[5] * t384 / 4.0) -
                                in2[5] * t385 / 4.0) -
                               in2[5] * t387 / 8.0) +
                              in2[5] * t156 / 8.0) +
                             in2[5] * d53 / 4.0) -
                            in2[5] * d54 / 8.0) +
                           in2[6] * d15 / 8.0) +
                          in2[6] * d19 / 8.0) -
                         in2[6] * d1 / 16.0) -
                        in2[6] * d6 / 16.0) +
                       in2[6] * d11 / 8.0)) +
                     (((((((((((((((((((((((((in2[6] * d37 * -0.0625 + in2[6] * d38 * 0.1875) - in2[7] * d56 / 4.0) - in2[6] * d41 / 16.0) +
                                           in2[6] * d42 * 0.1875) +
                                          in2[6] * d44 / 8.0) -
                                         in2[6] * t382 / 8.0) +
                                        in2[7] * d5 / 4.0) +
                                       in2[6] * t449 / 32.0) +
                                      in2[6] * t497 / 8.0) +
                                     in2[6] * t499 / 32.0) +
                                    in2[6] * t500 / 32.0) +
                                   in2[7] * d26 / 4.0) -
                                  in2[6] * t99 / 8.0) +
                                 in2[6] * d52 / 32.0) +
                                in2[7] * d45 / 8.0) +
                               in2[7] * t380 / 4.0) -
                              in2[7] * t381 / 4.0) -
                             in2[7] * t383 / 8.0) -
                            in2[7] * t498 / 8.0) -
                           in2[7] * d50 / 4.0) +
                          in2[7] * d51 / 8.0) -
                         in2[8] * d15 / 4.0) -
                        in2[8] * d11 / 4.0) +
                       in2[8] * d37 / 8.0) +
                      in2[8] * d41 / 8.0)) +
                    (((((((((t35 * 0.03022316 + in2[9] * d15 * 0.00212521) + in2[9] * d48 * 0.02635076) + in2[9] * d49 * 0.02824547) -
                          in2[9] * d * 0.02635076) -
                         in2[9] * d3 * 0.02824547) +
                        in2[9] * d4 * 0.02824547) +
                       in2[9] * d9 * 0.02635076) -
                      in2[9] * d11 * 0.004568035) -
                     in2[9] * d12 * 0.02824547)) +
                   (((((((((((((((((in2[9] * d16 * 0.00533377 - in2[9] * d28 * 0.02635076) - in2[9] * d37 * 0.001062605) -
                                  in2[9] * d40 * 0.00533377) -
                                 in2[9] * d41 * 0.001062605) -
                                d15 * t57 / 8.0) -
                               d15 * t58 / 8.0) +
                              d19 * t57 / 8.0) +
                             d15 * t59 / 4.0) -
                            d19 * t58 / 8.0) -
                           d1 * t57 / 16.0) +
                          d1 * t58 / 16.0) -
                         d6 * t57 / 16.0) +
                        d6 * t58 / 16.0) -
                       d11 * t57 / 8.0) -
                      d11 * t58 / 8.0) +
                     d11 * t59 / 4.0) +
                    d37 * t57 / 16.0)) +
                  ((((((((((((((((((((((((d38 * t57 * 0.1875 + d37 * t58 / 16.0) - d38 * t58 * 0.1875) - d37 * t59 / 8.0) + d41 * t57 / 16.0) +
                                      d42 * t57 * 0.1875) +
                                     d41 * t58 / 16.0) -
                                    d42 * t58 * 0.1875) -
                                   d41 * t59 / 8.0) +
                                  d44 * t57 / 8.0) -
                                 d44 * t58 / 8.0) -
                                t57 * t382 / 8.0) +
                               t58 * t382 / 8.0) +
                              t57 * t449 / 32.0) -
                             t58 * t449 / 32.0) +
                            t57 * t497 / 8.0) -
                           t58 * t497 / 8.0) +
                          t57 * t499 / 32.0) +
                         t57 * t500 / 32.0) -
                        t58 * t499 / 32.0) -
                       t58 * t500 / 32.0) -
                      t57 * t99 / 8.0) +
                     t58 * t99 / 8.0) +
                    d52 * t57 / 32.0) -
                   d52 * t58 / 32.0)) +
                 (((((((((in2[9] * std::cos(in1[1] + t11) * 0.35021932 - in2[9] * std::sin(ct_idx_96_tmp) * 0.07088939) -
                         in2[9] * std::sin(in1[1] + t33) * 0.07088939) -
                        in2[9] * std::sin(t11 + t33) * 0.06613412) +
                       in2[9] * std::cos(in1[1] * 2.0) * 0.187700645) +
                      in2[9] * std::cos(t11 * 2.0) * 0.16336328) +
                     t440_tmp * 0.1383) +
                    ct_idx_162_tmp * 0.1157) +
                   t36 * 0.3278) +
                  ct_idx_163_tmp * 0.1157)) +
                (((((((((((((t440_tmp * d15 * 0.0461 + t440_tmp * d48 * 0.2858) + t440_tmp * d49 * 0.30635) - t543_tmp * d56 * 0.02305) -
                          t389_tmp * d57 * 0.02305) -
                         t440_tmp * d * 0.2858) -
                        t440_tmp * d3 * 0.30635) +
                       t440_tmp * d4 * 0.30635) +
                      t543_tmp * d5 * 0.02305) -
                     t389_tmp * d7 * 0.02305) +
                    t440_tmp * d9 * 0.2858) +
                   t440_tmp * d11 * 0.0461) -
                  t440_tmp * d12 * 0.30635) +
                 t440_tmp * d16 * 0.05785)) +
               (((((((((((((t543_tmp * d26 * -0.005875 - t389_tmp * d27 * 0.005875) - t440_tmp * d28 * 0.2858) - t440_tmp * d37 * 0.02305) -
                         t440_tmp * d40 * 0.05785) -
                        t440_tmp * d41 * 0.02305) +
                       t543_tmp * d45 * 0.011525) +
                      t389_tmp * d47 * 0.011525) +
                     t543_tmp * t380 * 0.051975) -
                    t543_tmp * t381 * 0.051975) -
                   t389_tmp * t384 * 0.051975) -
                  t543_tmp * t383 * 0.011525) -
                 t389_tmp * t385 * 0.051975) -
                t389_tmp * t387 * 0.011525)) +
              (((((((((((((t543_tmp * t498 * -0.011525 + t389_tmp * t156 * 0.011525) + t543_tmp * d50 * 0.005875) + t543_tmp * d51 * 0.011525) -
                        t389_tmp * d53 * 0.005875) -
                       t389_tmp * d54 * 0.011525) +
                      t389_tmp * std::cos(t14) * 0.2858) -
                     t543_tmp * std::cos(t15) * 0.1639) +
                    t389_tmp * std::cos(t37) * 0.30635) +
                   t389_tmp * std::cos(t38) * 0.1429) +
                  t389_tmp * std::cos(ct_idx_238) * 0.153175) -
                 t389_tmp * std::cos(t81) * 0.2858) +
                t543_tmp * std::cos(ct_idx_320) * 0.1639) -
               t389_tmp * std::cos(ct_idx_327) * 0.30635)) +
             ((((((((((((t389_tmp * std::cos(t96) * 0.1429 + t389_tmp * std::cos(ct_idx_329) * 0.1429) + t389_tmp * std::cos(ct_idx_330) * 0.1429) +
                       t389_tmp * std::cos(ct_idx_15) * 0.30635) +
                      t389_tmp * std::cos(ct_idx_26) * 0.153175) +
                     t389_tmp * std::cos(ct_idx_27) * 0.153175) +
                    t389_tmp * std::cos(ct_idx_35) * 0.153175) +
                   t389_tmp * std::cos(ct_idx_51) * 0.2858) +
                  t389_tmp * std::cos(ct_idx_65) * 0.1429) -
                 t389_tmp * std::cos(ct_idx_78) * 0.30635) -
                t543_tmp * std::cos(ct_idx_84) * 0.05785) +
               t389_tmp * std::cos(ct_idx_86) * 0.153175) +
              t389_tmp * std::cos(ct_idx_87) * 0.153175)) +
            (((((((((((t389_tmp * std::cos(ct_idx_96) * 0.153175 - t389_tmp * std::cos(ct_idx_100) * 0.2858) +
                      t389_tmp * std::cos(ct_idx_123) * 0.1429) +
                     t389_tmp * std::cos(ct_idx_124) * 0.1429) -
                    t543_tmp * std::cos(t313) * 0.05785) +
                   t389_tmp * std::cos(ct_idx_161) * 0.153175) +
                  t389_tmp * std::cos(ct_idx_192) * 0.1429) -
                 t389_tmp * std::sin(t15) * 0.1639) -
                t543_tmp * std::sin(t14) * 0.2858) -
               t543_tmp * std::sin(t37) * 0.30635) -
              t543_tmp * std::sin(t38) * 0.1429) -
             t543_tmp * std::sin(ct_idx_238) * 0.153175)) +
           (((((((((((t389_tmp * std::sin(ct_idx_320) * -0.1639 - t543_tmp * std::sin(t81) * 0.2858) - t543_tmp * std::sin(ct_idx_327) * 0.30635) +
                    t543_tmp * std::sin(t96) * 0.1429) -
                   t543_tmp * std::sin(ct_idx_329) * 0.1429) -
                  t543_tmp * std::sin(ct_idx_330) * 0.1429) -
                 t543_tmp * std::sin(ct_idx_15) * 0.30635) +
                t543_tmp * std::sin(ct_idx_26) * 0.153175) -
               t543_tmp * std::sin(ct_idx_27) * 0.153175) -
              t543_tmp * std::sin(ct_idx_35) * 0.153175) -
             t543_tmp * std::sin(ct_idx_51) * 0.2858) -
            t543_tmp * std::sin(ct_idx_65) * 0.1429)) +
          (((((((((((((t543_tmp * std::sin(ct_idx_78) * -0.30635 - t389_tmp * std::sin(ct_idx_84) * 0.05785) +
                      t543_tmp * std::sin(ct_idx_86) * 0.153175) -
                     t543_tmp * std::sin(ct_idx_87) * 0.153175) +
                    t543_tmp * std::sin(ct_idx_96) * 0.153175) -
                   t543_tmp * std::sin(ct_idx_100) * 0.2858) +
                  t543_tmp * std::sin(ct_idx_123) * 0.1429) -
                 t543_tmp * std::sin(ct_idx_124) * 0.1429) +
                t389_tmp * std::sin(t313) * 0.05785) +
               t543_tmp * std::sin(ct_idx_161) * 0.153175) +
              t543_tmp * std::sin(ct_idx_192) * 0.1429) -
             t62_tmp * d56 / 4.0) -
            t65_tmp * d57 / 4.0) +
           t64_tmp * d2 / 8.0)) +
         ((((((((((((((((((((t62_tmp * d5 / 4.0 - t64_tmp * d8 / 8.0) - t65_tmp * d7 / 4.0) + t62_tmp * d26 / 4.0) + t65_tmp * d27 / 4.0) -
                         t64_tmp * d39 * 0.375) +
                        t64_tmp * d43 * 0.375) -
                       t64_tmp * d46 / 4.0) +
                      t62_tmp * d45 / 8.0) +
                     t65_tmp * d47 / 8.0) +
                    t62_tmp * t380 / 4.0) -
                   t62_tmp * t381 / 4.0) -
                  t65_tmp * t384 / 4.0) -
                 t62_tmp * t383 / 8.0) -
                t64_tmp * t386 / 4.0) -
               t65_tmp * t385 / 4.0) -
              t65_tmp * t387 / 8.0) -
             t64_tmp * t450 / 16.0) -
            t62_tmp * t498 / 8.0) -
           t64_tmp * t152 / 4.0) -
          t64_tmp * t143 / 16.0)) +
        (((((((t65_tmp * t156 / 8.0 + t64_tmp * t68 / 16.0) - t62_tmp * d50 / 4.0) + t62_tmp * d51 / 8.0) - t64_tmp * t132 / 4.0) +
           t65_tmp * d53 / 4.0) +
          t64_tmp * d55 / 16.0) -
         t65_tmp * d54 / 8.0);
    payload_inertia_regression_matrix[1] = t34;
    payload_inertia_regression_matrix[2] = t40;
    payload_inertia_regression_matrix[3] = t145;
    payload_inertia_regression_matrix[4] = t105;
    payload_inertia_regression_matrix[5] = t80;
    payload_inertia_regression_matrix[6] = t34;
    payload_inertia_regression_matrix[7] =
        ((((((((((((((((((((((((((((((((((((((((((((in2[6] + in2[9] * 0.72401518) + t57) + t59) + t83) + ct_idx_323) + ct_idx_324) + ct_idx_25) -
                                             t84) -
                                            t85) +
                                           ct_idx_38) +
                                          t161) +
                                         ct_idx_49) +
                                        ct_idx_62) +
                                       ct_idx_64) -
                                      t160) +
                                     ct_idx_90) +
                                    ct_idx_94) +
                                   ct_idx_98) +
                                  ct_idx_99) +
                                 ct_idx_103) +
                                ct_idx_116) +
                               ct_idx_130) +
                              ct_idx_145) +
                             ct_idx_162) +
                            ct_idx_163) -
                           ct_idx_131) +
                          ct_idx_193) +
                         ct_idx_198) +
                        ct_idx_211) +
                       ct_idx_213) +
                      ct_idx_217) +
                     ct_idx_221) +
                    ct_idx_226) +
                   ct_idx_235) +
                  ct_idx_273) +
                 ct_idx_282) +
                ct_idx_297) +
               ct_idx_280_tmp * 0.70043864) -
              ct_idx_307_tmp * 0.14177878) -
             ct_idx_308_tmp * 0.14177878) +
            ct_idx_303_tmp * 0.11298188) -
           ct_idx_312_tmp * 0.11298188) -
          b_ct_idx_260_tmp * 1.2254) -
         ct_idx_261_tmp * 1.2254) +
        (((((((b_ct_idx_239_tmp * 1.2254 - ct_idx_263_tmp * 1.2254) - ct_idx_264_tmp * 1.2254) - ct_idx_266_tmp * 1.2254) +
            b_t518_tmp * t4 * t5 * 1.2254) -
           ct_idx_284_tmp * 1.2254) -
          ct_idx_291_tmp * 1.2254) +
         ct_idx_226_tmp * t7 * t8 * t10 * 1.2254);
    payload_inertia_regression_matrix[8] = t479;
    payload_inertia_regression_matrix[9] = t314;
    payload_inertia_regression_matrix[10] = t294;
    payload_inertia_regression_matrix[11] = t480;
    payload_inertia_regression_matrix[12] = t40;
    payload_inertia_regression_matrix[13] = t479;
    payload_inertia_regression_matrix[14] = (((t477 + ct_idx_267) + ct_idx_273) + ct_idx_282) + ct_idx_297;
    payload_inertia_regression_matrix[15] = t13;
    payload_inertia_regression_matrix[16] = t126;
    payload_inertia_regression_matrix[17] = t175;
    payload_inertia_regression_matrix[18] = t145;
    payload_inertia_regression_matrix[19] = t314;
    payload_inertia_regression_matrix[20] = t13;
    payload_inertia_regression_matrix[21] = (((t60 + ct_idx_211) + ct_idx_226) + ct_idx_248) + ct_idx_297;
    payload_inertia_regression_matrix[22] = t51;
    payload_inertia_regression_matrix[23] = ct_idx_317;
    payload_inertia_regression_matrix[24] = t105;
    payload_inertia_regression_matrix[25] = t294;
    payload_inertia_regression_matrix[26] = t126;
    payload_inertia_regression_matrix[27] = t51;
    payload_inertia_regression_matrix[28] =
        (((((((((in2[3] + in2[9] * 0.00850084) + t58) + t59) - t83) + t85) - ct_idx_324) - ct_idx_25) + t160) - ct_idx_49) + ct_idx_98;
    payload_inertia_regression_matrix[29] = ct_idx_316;
    payload_inertia_regression_matrix[30] = t80;
    payload_inertia_regression_matrix[31] = t480;
    payload_inertia_regression_matrix[32] = t175;
    payload_inertia_regression_matrix[33] = ct_idx_317;
    payload_inertia_regression_matrix[34] = ct_idx_316;
    payload_inertia_regression_matrix[35] = (in2[8] + t57) + t58;
}

}  // namespace ur10

//
// File trailer for getPayloadInertiaRegressionMatrix.cpp
//
// [EOF]
//
