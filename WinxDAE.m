%
% Matlab code for the Course:
%
%     Modelling and Simulation of Mechatronics System
%
% by
% Enrico Bertolazzi
% Dipartimento di Ingegneria Industriale
% Universita` degli Studi di Trento
% email: enrico.bertolazzi@unitn.it
%
%TODOOOOOOOOOOOOOOOOOOOOO
classdef WinxDAE < DAE3baseClassImplicit
  properties (SetAccess = protected, Hidden = true)
    gravity;
    npos;
    nvel;
    L1 = 1.5*195-25; 
    L2 = 1.5*266-25;
    L3 = 1.5*130; 
    L4 = 1.5*59.3; 
    L5 = 1.5*175; 
    L6 = 1.5*207; 
    L7 = 1.5*64.9; 
    L8 = 1.5*64.9; 
    L9 = 120; 
    L10=40; 
    phi1=-pi/5;
    phi2=pi/4;

    m1 = 3000;
    m2 = 500;
    m3 = 500;
    m4 = 4000;
    m5 = 750;
    m6 = 150;
    IX2 = 0;     
    IX3 =0; 
    IX4 = 0;       
    IX5 = 0;      
    IZ1 =0;  
    IZ6 = 0;      


    c1 = 0;
    c2 = 0;
    c3 = 0;
    c4 = 0;
    c5 = 0;
    c6 = 10;
    prism=100;
  end

  % methods (Abstract)
  %   %
  %   %  Abstract functions defining an index-3 DAE with some derivatives
  %   %
  %   %  q' = v
  %   %  M(t,p) v' + Phi_p(t,q)^T lambda = gforce( t, q, v )
  %   %  Phi(t,q) = 0
  %   %
  %   %  d Phi(t,q) / dt     = A(t,q,v)
  %   %  d^2 Phi(t,q) / dt^2 = Phi_q(t,q) v - b(t,q,v)
  %   %
  %   % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  %   M( self, t, q )
  %   % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  %   % return function Phi(t,q)
  %   Phi( self, t, q )
  %   % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  %   % return function \partial Phi(t,q) / \partial t
  %   Phi_t( self, t, q )
  %   % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  %   % return function \partial Phi(t,q) / \partial q
  %   Phi_q( self, t, q )
  %   % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  %   % return function \partial Phi_q(t,q)*v / \partial q
  %   PhiV_q( self, t, q, v_dot )
  %   % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  %   % return function \partial Phi_q(t,q)^T*lambda / \partial q
  %   PhiL_q( self, t, q, lambda )
  %   % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  %   % return function \partial ( M(t,q) v_dot ) / \partial q
  %   W_q( self, t, q, v_dot )
  %   % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  %   gforce( self, t, q, v )
  %   % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  %   % return function \partial f( t, q, v ) / \partial q
  %   gforce_q( self, t, q, v )
  %   % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  %   % return function \partial f( t, q, v ) / \partial v
  %   gforce_v( self, t, q, v )
  %   % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  %   % d Phi(t,q) / dt     = Phi_q(t,q) v
  %   % d^2 Phi(t,q) / dt^2 = Phi_q(t,q) v' - b(t,q,v)
  %   b( self, t, q, v )
  %   % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  %   % return function \partial b( t, q, v ) / \partial q
  %   b_q( self, t, q, v )
  %   % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  %   % return function \partial b( t, q, v ) / \partial q
  %   b_v( self, t, q, v )
  %   % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  % end

  methods
    function self = WinxDAE( kappa, c, alpha )
      % 2 pos/vel, 1 constraints
      self@DAE3baseClassImplicit('WinxDAE',6,2);
      % self.I1    = 0.01;
      % self.R     = 0.1;
      % self.L     = 0.4;
      % self.x30   = 0.5;
      % self.m2    = 0.2;
      % self.H     = 0.05;
      % self.kappa = kappa;
      % self.c     = c;
      % self.alpha = (alpha*pi)/180;
    end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    function jac__Mass = M( self, t,  vars__ )
      L1=self.L1;
      L2=self.L2;
      L3=self.L3;
      L4=self.L4;
      L5=self.L5;
      L6=self.L6;
      L7=self.L7;
      L8=self.L8;
      L9=self.L9;
      L10=self.L10;
      phi1 = self.phi1; 
      phi2 = self.phi2;
      m1 = self.m1;
      m2 = self.m2;
      m3 = self.m3;
      m4 = self.m4;
      m5 = self.m5;
      m6 = self.m6;
      IZ1 = self.IZ1;
      IX2 = self.IX2;
      IX3 = self.IX3;
      IX4 = self.IX4;
      IX5 = self.IX5;
      IZ6 = self.IZ6;
      
      % extract states
      s = vars__(1);
      theta1 = vars__(2);
      theta2 = vars__(3);
      theta4 = vars__(4);
      theta5 = vars__(5);
      theta6 = vars__(6);
      % evaluate function
      jac__1_1 = m3;
      t1 = 2 * phi2;
      t2 = cos(t1);
      t3 = L9 ^ 2;
      t4 = t3 * m6;
      t5 = 4 * IZ6;
      t6 = t4 - t5;
      t8 = 2 * theta6;
      t9 = cos(t8);
      t10 = t9 * t6 * t2;
      t11 = sin(theta6);
      t12 = t11 * m6;
      t13 = L8 * L9;
      t14 = t13 * t12;
      t15 = 8 * t14;
      t16 = L8 ^ 2;
      t17 = 8 * t16;
      t18 = t17 + t3;
      t20 = m6 * t18 - t15 - t5;
      t22 = L7 * L9;
      t23 = cos(phi2);
      t27 = 8 * m6 * t11 * t23 * t22;
      t28 = m6 * L7;
      t29 = t23 * L8;
      t30 = t29 * t28;
      t31 = 16 * t30;
      t32 = L7 ^ 2;
      t33 = t32 * m5;
      t34 = 8 * t33;
      t35 = t32 * m6;
      t36 = 8 * t35;
      t37 = 8 * IX5;
      t39 = 2 * theta5;
      t40 = cos(t39);
      t42 = sin(t1);
      t44 = t9 * t6 * t42;
      t47 = -m6 * t18 + t15 + t5;
      t49 = sin(phi2);
      t50 = t49 * m6;
      t51 = L9 * t11;
      t53 = t51 - 2 * L8;
      t56 = 8 * t53 * L7 * t50;
      t58 = sin(t39);
      t64 = L6 ^ 2;
      t65 = m5 + m6;
      t66 = t65 * t64;
      t67 = 2 * phi1;
      t68 = cos(t67);
      t71 = cos(phi1);
      t72 = L6 * t71;
      t73 = t72 + L5;
      t74 = t23 * m6;
      t78 = t65 * L7;
      t81 = cos(theta5);
      t82 = t81 * (-t51 * t74 / 2 + L8 * t74 + t78) * t73;
      t85 = sin(theta5);
      t87 = L6 * t85 * t49 * t71;
      t88 = t85 * t49;
      t89 = L5 * t88;
      t90 = L7 * t23;
      t98 = t49 * L8;
      t103 = t71 * (-t98 * t85 * m6 + t65 * L5) * L6;
      t107 = m6 * t88 * L5 * L8;
      t109 = L6 * L9;
      t110 = sin(phi1);
      t111 = cos(theta6);
      t114 = m6 * t111 * t110 * t109;
      t116 = L5 ^ 2;
      t117 = 16 * t116;
      t118 = 8 * t32;
      t124 = t116 * (64 * m4 + 16 * m5);
      t126 = t40 * (t2 * t20 - t10 - t27 + t31 + t34 + t36 - t37) + t58 * (t42 * t47 + t44 + t56) + t9 * (-3 * t4 + 12 * IZ6) + 16 * t68 * t66 + 32 * t82 + 16 * t51 * (t87 + t89 - t90 / 2 - L8 / 2) * m6 + t31 + 32 * t103 - 32 * t107 + 16 * t114 + m6 * (t117 + t118 + t17 - t3) + t34 + t124 - 16 * IX4 - t37 + t5;
      t127 = 2 * theta4;
      t128 = cos(t127);
      t136 = t23 * t81 - t88;
      t138 = sin(t8);
      t141 = sin(t67);
      t149 = L8 * m6;
      t150 = L6 * t110;
      t153 = t150 - L9 * t111 / 2;
      t159 = t111 * m6;
      t162 = t110 * t65 * L6 - L9 * t159 / 2;
      t163 = L7 * t162;
      t183 = sin(t127);
      t190 = -t6;
      t195 = (s + L3 + L4 / 2) ^ 2;
      t198 = L3 ^ 2;
      t199 = t198 * m2;
      t205 = cos(2 * theta2);
      t208 = L9 * m6;
      t217 = t128 * t126 / 32 + t40 * (t2 * t47 + t10 + t27 - t31 - t34 - t36 + t37) / 32 + t183 * (-4 * t138 * t6 * t136 - 16 * t141 * t66 + t81 * (16 * L9 * L6 * t11 * t110 * t74 - 32 * t23 * t153 * t149 - 32 * t163) - 16 * t12 * t85 * t110 * t49 * t109 + 16 * m6 * t111 * t71 * t109 + 32 * t49 * t85 * t153 * t149 - 32 * L5 * t162) / 32 + t58 * (t42 * t20 - t44 - t56) / 32 + t9 * t190 / 32 + t205 * (16 * m3 * t195 - 16 * IX2 - 16 * IX3 + 4 * t199) / 32 + t82 + t11 * (t87 + t89 - 0.3e1 / 0.2e1 * t90 - 0.3e1 / 0.2e1 * L8) * t208 / 2 + 0.3e1 / 0.2e1 * t30 + t103 - t107 - t114 / 2;
      t225 = m5 * t64;
      t235 = s ^ 2;
      t237 = s * L4;
      t239 = L4 ^ 2;
      t249 = m6 * (t117 + 16 * t64 + 24 * t32 + 24 * t16 + 5 * t3) / 32 + t225 / 2 + 0.3e1 / 0.4e1 * t33 + t124 / 32 + 0.3e1 / 0.8e1 * IZ6 + m3 * (16 * t198 + (32 * s + 16 * L4) * L3 + 16 * t235 + 16 * t237 + 4 * t239) / 32 + t199 / 8 + IX2 / 2 + IX3 / 2 + IX4 / 2 + IX5 / 4 + IZ1;
      jac__2_2 = t217 + t249;
      t250 = L6 * L8;
      t252 = cos(theta4 + phi1 - theta5 - phi2);
      t254 = t252 * m6 * t250;
      t257 = cos(theta4 + phi1 + theta5 + phi2);
      t259 = t257 * m6 * t250;
      t261 = m6 * L5;
      t263 = cos(phi2 - theta4 + theta5);
      t265 = t263 * L8 * t261;
      t268 = sin(phi2 + theta6 + phi1 + theta4 + theta5);
      t270 = t268 * m6 * t109;
      t273 = sin(-phi2 + theta6 + phi1 + theta4 - theta5);
      t275 = t273 * m6 * t109;
      t278 = sin(phi2 - theta6 + phi1 + theta4 + theta5);
      t280 = t278 * m6 * t109;
      t283 = sin(-phi2 - theta6 + phi1 + theta4 - theta5);
      t285 = t283 * m6 * t109;
      t288 = sin(theta6 + theta4 + theta5);
      t290 = t288 * m6 * t22;
      t293 = sin(-theta6 + theta4 - theta5);
      t295 = t293 * m6 * t22;
      t298 = sin(-theta6 + theta4 + theta5);
      t300 = t298 * m6 * t22;
      t303 = sin(theta6 + theta4 - theta5);
      t305 = t303 * m6 * t22;
      t307 = L6 * L7;
      t309 = cos(phi1 + theta4 + theta5);
      t314 = cos(phi1 + theta4 - theta5);
      t318 = L8 + L5;
      t320 = sin(-theta4 + theta5 + phi2 + theta6);
      t325 = sin(theta4 + theta5 + phi2 - theta6);
      t329 = -L8 + L5;
      t331 = sin(theta4 + theta5 + phi2 + theta6);
      t335 = L5 * L7;
      t337 = cos(theta4 - theta5);
      t342 = sin(-theta4 + theta5 + phi2 - theta6);
      t347 = 16 * t33 + 16 * t35 - 16 * IX5;
      t349 = cos(theta4 - t39);
      t354 = 16 * t16 + 2 * t3;
      t356 = 8 * IZ6;
      t359 = cos(-theta4 + t39 + t1);
      t362 = t254 / 2 - t259 / 2 + t265 / 2 + t270 / 8 - t275 / 8 - t280 / 8 + t285 / 8 - t290 / 8 + t295 / 8 - t300 / 8 + t305 / 8 - t309 * t65 * t307 / 2 + t314 * t65 * t307 / 2 - t320 * t318 * t208 / 8 - t325 * t318 * t208 / 8 + t331 * t329 * t208 / 8 + t337 * t65 * t335 / 2 + t342 * t329 * t208 / 8 + t349 * t347 / 64 + t359 * (m6 * t354 - t356) / 64;
      t364 = -2 * t4 + t356;
      t366 = cos(-theta4 + t8 + theta5 + phi2);
      t373 = cos(theta4 + t39 + t1);
      t378 = cos(theta4 + t39);
      t382 = cos(phi2 + theta4 + theta5);
      t385 = cos(theta4 + theta5);
      t391 = -t364;
      t393 = cos(-t8 - theta4 + theta5 + phi2);
      t397 = cos(theta4 + t39 + t1 + t8);
      t401 = cos(theta4 + t39 + t1 - t8);
      t405 = cos(-theta4 + t39 + t1 - t8);
      t409 = cos(t1 + t8 + t39 - theta4);
      t413 = cos(-t8 + theta4 + theta5 + phi2);
      t418 = cos(t8 + theta4 + theta5 + phi2);
      t422 = sin(theta4 + t39 + phi2 - theta6);
      t424 = t422 * m6 * t22;
      t427 = sin(-theta4 + t39 + phi2 - theta6);
      t429 = t427 * m6 * t22;
      t432 = sin(theta4 + t39 + phi2 + theta6);
      t434 = t432 * m6 * t22;
      t437 = sin(-theta4 + t39 + phi2 + theta6);
      t439 = t437 * m6 * t22;
      t442 = sin(theta4 + theta6 + t39 + t1);
      t444 = t442 * L9 * t149;
      t447 = sin(-theta4 + theta6 + t39 + t1);
      t449 = t447 * L9 * t149;
      t452 = sin(theta4 - theta6 + t39 + t1);
      t454 = t452 * L9 * t149;
      t457 = sin(-theta4 - theta6 + t39 + t1);
      t459 = t457 * L9 * t149;
      t462 = cos(-theta4 + t39 + phi2);
      t464 = t462 * L8 * t28;
      t467 = cos(theta4 + t39 + phi2);
      t469 = t467 * L8 * t28;
      t471 = t418 * t364 / 64 - t424 / 8 + t429 / 8 + t434 / 8 - t439 / 8 + t444 / 8 - t449 / 8 - t454 / 8 + t459 / 8 + t464 / 2 - t469 / 2;
      jac__2_4 = t362 + t366 * t364 / 64 + t373 * (-m6 * t354 + t356) / 64 - t378 * t347 / 64 - (t65 * L7 * t385 + t382 * t149) * L5 / 2 + t393 * t391 / 64 + t397 * t6 / 64 + t401 * t6 / 64 + t405 * t190 / 64 + t409 * t190 / 64 + t413 * t391 / 64 + t471;
      t473 = sin(theta4);
      t478 = cos(theta4);
      t507 = (L9 * L8 * t111 - 2 * L8 * t150 + t51 * t150) * m6;
      jac__2_5 = -t138 * t136 * t6 * t473 / 8 - t9 * t6 * t478 / 8 + t478 * (-4 * t81 * t73 * (t23 * t53 * m6 - 2 * t78) - 8 * t23 * t53 * t28 + 4 * t85 * t73 * t53 * t50 - t15 + m6 * (t118 + t17 + t3) + t34 + t5) / 8 + (t81 * (t23 * t507 - 2 * t163) - t507 * t88) * t473 / 2;
      t516 = t283 * L6;
      t517 = t273 * L6;
      t518 = t342 * t318;
      t519 = t278 * L6;
      t520 = t293 * L7;
      t522 = sin(-theta4 + phi2 - theta6);
      t523 = t522 * L7;
      t526 = t268 * L6;
      t527 = t298 * L7;
      t528 = t303 * L7;
      t529 = t263 * L9;
      t531 = sin(-theta4 + phi2 + theta6);
      t532 = t531 * L7;
      t534 = sin(theta4 + phi2 - theta6);
      t535 = t534 * L7;
      t536 = t331 * t318;
      t537 = t382 * L9;
      t539 = sin(theta6 + phi2 + theta4);
      t540 = t539 * L7;
      t541 = t288 * L7;
      t542 = t320 * t329 + t325 * t329 - t516 - t517 + t518 + t519 + t520 + t523 + t526 - t527 - t528 + t529 + t532 + t535 + t536 - t537 + t540 + t541;
      jac__2_6 = -m6 * t542 * L9 / 8;
      jac__3_3 = m3 * (t198 + (L4 + 0.2e1 * s) * L3 + 0.2500000000e0 * t239 + t235 + t237) + 0.2500000000e0 * t199;
      t556 = 0.1250000000e0 * L8;
      t557 = 0.1250000000e0 * L5;
      t559 = L9 * (-t556 - t557);
      t565 = L9 * (-t556 + t557);
      t582 = 0.1250000000e0 * IZ6;
      t584 = -t582 + 0.3125000000e-1 * t4;
      t588 = -0.3125000000e-1 * t3 - 0.2500000000e0 * t16;
      t595 = 0.2500000000e0 * t33 + 0.2500000000e0 * t35 - 0.2500000000e0 * IX5;
      t601 = -t584;
      t603 = -0.1250000000e0 * t280 + 0.1250000000e0 * t285 - 0.1250000000e0 * t290 + 0.1250000000e0 * t295 - 0.1250000000e0 * t300 + 0.1250000000e0 * t305 + t413 * t584 + t373 * (m6 * t588 + t582) + t349 * t595 + t359 * (-m6 * t588 - t582) + t366 * t601;
      t606 = m6 * t335;
      t608 = m5 * t335;
      t610 = -0.5000000000e0 * t606 - 0.5000000000e0 * t608;
      t612 = m5 * t307;
      t614 = L6 * m6;
      t615 = L7 * t614;
      t617 = -0.5000000000e0 * t612 - 0.5000000000e0 * t615;
      t621 = 0.6250000000e-1 * IZ6 - 0.1562500000e-1 * t4;
      t628 = -t621;
      t645 = t418 * t601 - 0.1250000000e0 * t424 + 0.1250000000e0 * t429 + 0.1250000000e0 * t434 - 0.1250000000e0 * t439 + 0.1250000000e0 * t444 - 0.1250000000e0 * t449 - 0.1250000000e0 * t454 + 0.1250000000e0 * t459 + 0.5000000000e0 * t464 - 0.5000000000e0 * t469;
      jac__4_2 = -0.5000000000e0 * t382 * L8 * t261 + t320 * m6 * t559 + t325 * m6 * t559 + t342 * m6 * t565 + t331 * m6 * t565 + 0.5000000000e0 * t254 - 0.5000000000e0 * t259 + 0.5000000000e0 * t265 + 0.1250000000e0 * t270 - 0.1250000000e0 * t275 + t603 + t309 * t617 - t314 * t617 - t337 * t610 - t378 * t595 + t385 * t610 + t393 * t584 + t397 * t628 + t401 * t628 + t405 * t621 + t409 * t621 + t645;
      t648 = sin(t39 + phi2 + theta6);
      t653 = sin(t39 + phi2 - theta6);
      t658 = sin(theta6 + t39 + t1);
      t663 = sin(-theta6 + t39 + t1);
      t667 = phi2 - theta6 + theta5;
      t668 = sin(t667);
      t669 = t668 * L9;
      t672 = theta5 + phi2;
      t673 = cos(t672);
      t677 = phi2 + theta6 + theta5;
      t678 = sin(t677);
      t679 = t678 * L9;
      t683 = cos(t39 + phi2);
      t687 = sin(theta6 + phi1);
      t692 = sin(-theta6 + phi1);
      t697 = cos(phi1 - theta5 - phi2);
      t699 = t697 * m6 * t250;
      t701 = cos(phi1 + theta5 + phi2);
      t703 = t701 * m6 * t250;
      t705 = sin(phi2 + theta6);
      t710 = sin(phi2 - theta6);
      t715 = sin(theta5 + phi2 + theta6 + phi1);
      t717 = t715 * m6 * t109;
      t720 = sin(theta5 + phi2 - theta6 + phi1);
      t722 = t720 * m6 * t109;
      t725 = sin(-theta5 - phi2 + theta6 + phi1);
      t727 = t725 * m6 * t109;
      t729 = -0.2500000000e0 * t648 * m6 * t22 + 0.2500000000e0 * t653 * m6 * t22 - 0.2500000000e0 * t658 * L9 * t149 + 0.2500000000e0 * t663 * L9 * t149 + 0.5000000000e0 * t669 * t261 + 0.2000000000e1 * t673 * L8 * t261 - 0.5000000000e0 * t679 * t261 + t683 * L8 * t28 - 0.5000000000e0 * t687 * m6 * t109 - 0.5000000000e0 * t692 * m6 * t109 + t699 + t703 - 0.2500000000e0 * t705 * m6 * t22 + 0.2500000000e0 * t710 * m6 * t22 - 0.2500000000e0 * t717 + 0.2500000000e0 * t722 - 0.2500000000e0 * t727;
      t731 = sin(-theta5 - phi2 - theta6 + phi1);
      t733 = t731 * m6 * t109;
      t736 = cos(theta5 + phi1);
      t740 = cos(-theta5 + phi1);
      t743 = 0.5000000000e0 * IX5;
      t744 = 0.2500000000e0 * IZ6;
      t767 = cos(2 * t672);
      t773 = cos(2 * t667);
      t776 = cos(2 * t677);
      t782 = 0.5000000000e0 * t33;
      t786 = 0.2500000000e0 * t733 + t736 * t65 * t307 + t740 * t65 * t307 + t743 + t744 + t81 * (0.2e1 * t606 + 0.2e1 * t608) + t71 * (0.2e1 * L6 * t261 + 0.2e1 * L5 * L6 * m5) + m6 * (0.5e0 * t16 + 0.1875e0 * t3 + t64 + t116 + 0.5000000000e0 * t32) + t767 * (m6 * (0.5000000000e0 * t16 + 0.6250000000e-1 * t3) - t744) + t116 * (0.4e1 * m4 + m5) + t773 * t601 + t776 * t601 + t9 * (0.6250000000e-1 * t4 - t744) + t40 * (0.5000000000e0 * t35 + t782 - t743) + t225 + t782 - 0.5000000000e0 * t14 + t30;
      jac__4_4 = t729 + t786;
      t788 = cos(-t8 + theta5 + phi2);
      t792 = cos(t8 + theta5 + phi2);
      t796 = sin(theta5 + theta6);
      t801 = sin(theta5 - theta6);
      t811 = t740 * L7;
      jac__4_5 = -0.2500000000e0 * IZ6 * t788 + 0.2500000000e0 * IZ6 * t792 - 0.2500000000e0 * t796 * m6 * t22 - 0.2500000000e0 * t801 * m6 * t22 + 0.1250000000e0 * t733 + t699 / 2 - t703 / 2 - t736 * m5 * t307 / 2 + t811 * t614 / 2 - t736 * L7 * t614 / 2 + t811 * L6 * m5 / 2 - 0.2500000000e0 * t679 * t149 - 0.2500000000e0 * t669 * t149 - 0.1250000000e0 * t722 - 0.1250000000e0 * t727 + 0.1250000000e0 * t717 + 0.6250000000e-1 * t788 * t4 - 0.6250000000e-1 * t792 * t4;
      t837 = t111 * t150;
      t838 = 0.5e0 * L9;
      jac__4_6 = 0.5e0 * m6 * L9 * (t11 * (t81 * (L7 + t29) + L5 - 0.1e1 * t85 * t98 + t72) + t81 * t23 * (t837 - t838) + t85 * t49 * (-0.1e1 * t837 + t838));
      t849 = 0.5000000000e0 * IZ6;
      t851 = t849 - 0.1250000000e0 * t4;
      t857 = t473 * t85;
      t868 = -0.5000000000e0 * L9 * t261 - 0.5000000000e0 * t71 * L9 * t614;
      t899 = 0.5000000000e0 * t159 * t13;
      t901 = m6 * t110 * t250;
      t906 = 0.5000000000e0 * m6 * t11 * t110 * t109;
      jac__5_2 = t138 * (t81 * t473 * t23 * t851 - t857 * t49 * t851) + t9 * t478 * t851 + t478 * (t81 * (t23 * (t11 * t868 + t73 * t149) + t73 * t78) + t23 * (0.2000000000e1 * L8 * t28 - 0.1e1 * t12 * t22) + t85 * t49 * (-t11 * t868 - 0.1e1 * m6 * t71 * t250 - 0.1e1 * L8 * t261) - 0.1e1 * t14 + m6 * (t32 + 0.1250000000e0 * t3 + t16) + t849 + t33) + t81 * t473 * (t23 * (t899 - 0.1e1 * t901 + t906) + t110 * (-0.1e1 * t612 - 0.1e1 * t615) + 0.5000000000e0 * t159 * t22) + t857 * t49 * (t901 - t899 - t906);
      jac__5_4 = jac__4_5;
      jac__5_5 = t9 * t851 + t11 * (-0.1e1 * L9 * t149 - 0.1e1 * t23 * L9 * t28) + 0.2e1 * t30 + m6 * (t32 + t16 + 0.125e0 * t3) + t33 + t849;
      jac__5_6 = -0.25e0 * (t705 + t710) * m6 * t22;
      t940 = L5 - 0.1e1 * L8;
      t946 = -0.1e1 * t516 - 0.1e1 * t517 + t518 + t519 + t520 + t523 + t320 * t940 + t325 * t940 + t526 - 0.1e1 * t527 - 0.1e1 * t528 + t529 + t532 + t535 + t536 - 0.1e1 * t537 + t540 + t541;
      jac__6_2 = -0.125e0 * m6 * L9 * t946;
      jac__6_4 = jac__4_6;
      jac__6_5 = jac__5_6;
      jac__6_6 = 0.25e0 * t4;
      
      % store on output
      jac__Mass = zeros(6,6);
      jac__Mass(1,1) = jac__1_1;
      jac__Mass(2,2) = jac__2_2;
      jac__Mass(2,4) = jac__2_4;
      jac__Mass(2,5) = jac__2_5;
      jac__Mass(2,6) = jac__2_6;
      jac__Mass(3,3) = jac__3_3;
      jac__Mass(4,2) = jac__4_2;
      jac__Mass(4,4) = jac__4_4;
      jac__Mass(4,5) = jac__4_5;
      jac__Mass(4,6) = jac__4_6;
      jac__Mass(5,2) = jac__5_2;
      jac__Mass(5,4) = jac__5_4;
      jac__Mass(5,5) = jac__5_5;
      jac__Mass(5,6) = jac__5_6;
      jac__Mass(6,2) = jac__6_2;
      jac__Mass(6,4) = jac__6_4;
      jac__Mass(6,5) = jac__6_5;
      jac__Mass(6,6) = jac__6_6;
    end

    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    function res__Phi = Phi( self, t, vars__ )
      L1=self.L1;
      L2=self.L2;
      L3=self.L3;
      L4=self.L4;
      L5=self.L5;
      L6=self.L6;
      L7=self.L7;
      L8=self.L8;
      L9=self.L9;
      L10=self.L10;
      phi1 = self.phi1; 
      phi2 = self.phi2;
      m1 = self.m1;
      m2 = self.m2;
      m3 = self.m3;
      m4 = self.m4;
      m5 = self.m5;
      m6 = self.m6;
      IZ1 = self.IZ1;
      IX2 = self.IX2;
      IX3 = self.IX3;
      IX4 = self.IX4;
      IX5 = self.IX5;
      IZ6 = self.IZ6;

      % extract states
      s = vars__(1);
      theta1 = vars__(2);
      theta2 = vars__(3);
      theta4 = vars__(4);
      theta5 = vars__(5);
      theta6 = vars__(6);
      % evaluate function
      t1 = -s - L3 - L4;
      t2 = cos(theta2);
      t4 = cos(theta4);
      res__1 = t4 * L5 + t2 * t1;
      t6 = sin(theta2);
      t8 = sin(theta4);
      res__2 = L5 * t8 + t6 * t1 - L1 + L2;
      
      % store on output
      res__Phi = zeros(2,1);
      res__Phi(1) = res__1;
      res__Phi(2) = res__2;
    end

    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    function rhs = Phi_t( self, t, vars__ )
      rhs = zeros(2,1);
    end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    function jac__DPhiDp = Phi_q( self, t, vars__ )
      L1=self.L1;
      L2=self.L2;
      L3=self.L3;
      L4=self.L4;
      L5=self.L5;
      L6=self.L6;
      L7=self.L7;
      L8=self.L8;
      L9=self.L9;
      L10=self.L10;
      phi1 = self.phi1; 
      phi2 = self.phi2;
      m1 = self.m1;
      m2 = self.m2;
      m3 = self.m3;
      m4 = self.m4;
      m5 = self.m5;
      m6 = self.m6;
      IZ1 = self.IZ1;
      IX2 = self.IX2;
      IX3 = self.IX3;
      IX4 = self.IX4;
      IX5 = self.IX5;
      IZ6 = self.IZ6;

      % extract states
      s = vars__(1);
      theta1 = vars__(2);
      theta2 = vars__(3);
      theta4 = vars__(4);
      theta5 = vars__(5);
      theta6 = vars__(6);
      % evaluate function
      t1 = cos(theta2);
      jac__1_1 = -t1;
      t2 = L3 + L4 + s;
      t3 = sin(theta2);
      jac__1_3 = t3 * t2;
      t4 = sin(theta4);
      jac__1_4 = -t4 * L5;
      jac__2_1 = -t3;
      jac__2_3 = -t2 * t1;
      t7 = cos(theta4);
      jac__2_4 = t7 * L5;
      
      % store on output
      jac__DPhiDp = zeros(2,6);
      jac__DPhiDp(1,1) = jac__1_1;
      jac__DPhiDp(1,3) = jac__1_3;
      jac__DPhiDp(1,4) = jac__1_4;
      jac__DPhiDp(2,1) = jac__2_1;
      jac__DPhiDp(2,3) = jac__2_3;
      jac__DPhiDp(2,4) = jac__2_4;
    end

    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % function H = PhiL_q( self, t, pos, lambda )
    %   theta1 = pos(1);
    %   x2     = pos(2);
    %   alpha  = self.alpha;
    %   R      = self.R;
    %   H      = [ R*cos(alpha+theta1), 0; 0, 0];
    % end
    % % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % function H = PhiV_q( self, t, pos, v_dot )
    %   theta1 = pos(1);
    %   x2     = pos(2);
    %   alpha  = self.alpha;
    %   R      = self.R;
    %   H      = [ R*cos(alpha+theta1)*v_dot(1), 0 ];
    % end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    function res__gfun = gforce( self, t, pos, vel )
      vars__=[pos, vel];

      L1=self.L1;
      L2=self.L2;
      L3=self.L3;
      L4=self.L4;
      L5=self.L5;
      L6=self.L6;
      L7=self.L7;
      L8=self.L8;
      L9=self.L9;
      L10=self.L10;
      phi1 = self.phi1; 
      phi2 = self.phi2;
      m1 = self.m1;
      m2 = self.m2;
      m3 = self.m3;
      m4 = self.m4;
      m5 = self.m5;
      m6 = self.m6;
      IZ1 = self.IZ1;
      IX2 = self.IX2;
      IX3 = self.IX3;
      IX4 = self.IX4;
      IX5 = self.IX5;
      IZ6 = self.IZ6;
      c1=self.c1;
      c5=self.c5;
      c6=self.c6;
      prism=self.prism;

      % extract states
      s = vars__(1);
      theta1 = vars__(2);
      theta2 = vars__(3);
      theta4 = vars__(4);
      theta5 = vars__(5);
      theta6 = vars__(6);
      s__dot = vars__(7);
      theta1__dot = vars__(8);
      theta2__dot = vars__(9);
      theta4__dot = vars__(10);
      theta5__dot = vars__(11);
      theta6__dot = vars__(12);
      % evaluate function
      t3 = theta1__dot ^ 2;
      t5 = cos(theta2);
      t6 = t5 ^ 2;
      t9 = sin(theta2);
      t14 = theta2__dot ^ 2;
      res__1 = t6 * m3 * t3 * (s + L3 + 0.5000000000e0 * L4) - 0.9810000000e1 * m3 * t9 + m3 * t14 * (s + L3 + 0.5e0 * L4) + prism;
      t17 = L9 ^ 2;
      t18 = t17 * m6;
      t20 = IZ6 / 2;
      t21 = t18 / 8 - t20;
      t22 = 2 * theta6;
      t23 = cos(t22);
      t25 = sin(theta6);
      t26 = t25 * m6;
      t27 = L8 * L9;
      t28 = t27 * t26;
      t29 = L8 ^ 2;
      t30 = t17 / 8;
      t31 = -t29 - t30;
      t32 = m6 * t31;
      t33 = t23 * t21 + t20 + t28 + t32;
      t34 = 2 * phi2;
      t35 = sin(t34);
      t37 = sin(phi2);
      t38 = t37 * m6;
      t39 = L9 * t25;
      t41 = t39 - 2 * L8;
      t42 = t41 * L7;
      t46 = 2 * theta5;
      t47 = sin(t46);
      t55 = cos(t34);
      t57 = m6 * L7;
      t58 = cos(phi2);
      t61 = L7 ^ 2;
      t62 = t61 * m5;
      t63 = t61 * m6;
      t66 = cos(t46);
      t70 = cos(theta5);
      t73 = t37 * theta6__dot;
      t74 = sin(theta5);
      t79 = t18 - 4 * IZ6;
      t84 = t37 * t70;
      t87 = sin(t22);
      t88 = t87 * t79;
      t91 = L6 ^ 2;
      t92 = t91 * theta4__dot;
      t93 = m5 + m6;
      t94 = 2 * phi1;
      t95 = cos(t94);
      t99 = m6 * theta5__dot;
      t100 = sin(phi1);
      t102 = L6 * L9;
      t104 = L6 * t100;
      t105 = L8 * t104;
      t107 = cos(theta6);
      t109 = L9 * L8 * t107;
      t110 = t102 * t25 * t100 - 2 * t105 + t109;
      t114 = cos(phi1);
      t116 = L6 * t114 * theta4__dot;
      t117 = theta4__dot * L5;
      t118 = theta6__dot * L8;
      t128 = theta4__dot * L8;
      t130 = L6 * t114 + L5;
      t133 = t25 * (t116 + t117 - t118 / 2) * L9 + L9 * L6 * t107 * t100 * theta6__dot / 2 - 2 * t130 * t128;
      t137 = t107 * m6;
      t138 = L9 * t137;
      t141 = -t138 / 2 + t93 * t104;
      t148 = theta4__dot * L7;
      t149 = t41 * t148;
      t159 = m6 * theta6__dot;
      t168 = L9 * m6;
      t170 = L6 * t114 * theta6__dot;
      t171 = theta6__dot * L5;
      t176 = t107 * t100;
      t180 = L5 * L6;
      t184 = L5 ^ 2;
      t200 = t47 * theta4__dot * (t35 * t33 + t42 * t38) / 8 + t66 * (t55 * (-m6 * t31 - t23 * t21 - t20 - t28) - t58 * t41 * t57 + t62 + t63 - IX5) * theta4__dot / 8 - 0.3e1 / 0.64e2 * t23 * t79 * (-0.4e1 / 0.3e1 * t70 * t58 * theta6__dot + 0.4e1 / 0.3e1 * t74 * t73 + theta4__dot) - t88 * (t58 * t74 + t84) * theta5__dot / 32 + t95 * t93 * t92 / 4 + t74 * (t58 * t110 * t99 / 8 + t37 * m6 * t133 / 4 - L7 * theta5__dot * t141 / 4) - t58 * m6 * (t70 * t133 + t149 / 2) / 4 + t37 * t110 * t70 * t99 / 8 + t70 * L7 * (t39 * t159 / 4 + t93 * t130 * theta4__dot) / 2 - t25 * (-t170 + t128 - t171) * t168 / 8 + (m6 * t176 * t102 / 4 + t114 * t93 * t180 / 2 + m6 * (t184 / 4 + t61 / 8 + t29 / 8 - t17 / 64) + t62 / 8 + IZ6 / 16 + t184 * (m4 + m5 / 4) - IX4 / 4 - IX5 / 8) * theta4__dot;
      t202 = 2 * theta4;
      t203 = sin(t202);
      t206 = theta5__dot * t33;
      t212 = t107 * L9;
      t213 = L8 * m6;
      t216 = (t87 * (-t18 / 4 + IZ6) + t213 * t212) * theta6__dot;
      t222 = t107 * t159;
      t223 = L9 * t37;
      t224 = L7 * t223;
      t225 = t224 * t222;
      t227 = t62 + t63 - IX5;
      t228 = t227 * theta5__dot;
      t236 = t107 * theta6__dot;
      t237 = L9 * t58;
      t240 = t37 * theta5__dot;
      t248 = t58 * theta4__dot;
      t257 = sin(t94);
      t261 = t130 * t41;
      t265 = t100 * theta4__dot;
      t274 = L6 * L8;
      t277 = L9 * L6 * t25 * t265 + t107 * (-t170 / 2 + t128 - t171 / 2) * L9 - 2 * t274 * t265;
      t288 = L9 * L7;
      t303 = L9 * t100;
      t304 = L6 * t303;
      t314 = t47 * (-t55 * t206 / 4 - t35 * t216 / 8 - t58 * t42 * t99 / 4 - t225 / 8 + t228 / 4) + t66 * (t55 * t216 / 8 - t35 * t206 / 4 - L7 * (-t237 * t236 / 2 + t41 * t240) * m6 / 4) + t87 * t79 * (t70 * t248 - t74 * t37 * theta4__dot - 0.3e1 / 0.4e1 * theta6__dot) / 8 + t257 * t93 * t92 / 2 + t74 * (-t58 * t261 * t99 / 4 + t37 * m6 * t277 / 2 + t93 * t130 * theta5__dot * L7 / 2) - t58 * (t70 * t277 - t288 * t236 / 4) * m6 / 2 - t37 * t261 * t70 * t99 / 4 + t70 * L7 * t141 * theta4__dot + t304 * t25 * t159 / 4 - t107 * (t116 + t117 - t118 / 4) * t168 / 2 + t93 * t180 * t265;
      t316 = cos(t202);
      t319 = theta4__dot ^ 2;
      t320 = cos(theta4);
      t322 = theta1__dot * theta5__dot;
      t323 = t320 * t319 + t322;
      t326 = theta4__dot * theta6__dot;
      t327 = sin(theta4);
      t328 = t79 * t327;
      t332 = theta4__dot * m6;
      t333 = theta6__dot * t332;
      t338 = t28 + t32 + t20;
      t341 = 8 * t27 * t327 * t107 * t333 + t23 * t79 * t323 - 2 * t87 * t328 * t326 + 8 * t338 * t323;
      t343 = theta4__dot * theta5__dot;
      t347 = theta1__dot * theta6__dot;
      t350 = t327 * theta5__dot;
      t354 = t138 * L8 * t347;
      t356 = -2 * t23 * t328 * t343 - 16 * t350 * t338 * theta4__dot - t88 * t347 + 4 * t354;
      t363 = t58 * L7;
      t366 = theta5__dot * t332;
      t368 = t327 * t41;
      t372 = t347 * t137;
      t373 = t372 * t224;
      t407 = theta5__dot * theta6__dot;
      t408 = t327 * t70;
      t424 = (theta4__dot - theta5__dot) * (theta4__dot + theta5__dot);
      t431 = t320 * theta5__dot;
      t440 = s + L3 + L4 / 2;
      t441 = t440 ^ 2;
      t443 = L3 ^ 2;
      t444 = t443 * m2;
      t448 = 2 * theta2;
      t449 = sin(t448);
      t452 = t440 * s__dot;
      t453 = m3 * theta1__dot;
      t454 = cos(t448);
      t457 = theta5__dot ^ 2;
      t458 = theta6__dot ^ 2;
      t459 = t319 - t457 - t458;
      t463 = t319 - t457 + t458;
      t471 = t25 * t459 * L9 * t104 - 2 * (-t107 * t463 * L9 / 2 + t424 * t104) * L8;
      t475 = L5 * t319;
      t476 = L8 * t326;
      t485 = t25 * (t320 * (-t114 * t459 * L6 - t475 + 2 * t476 + (t457 + t458) * L5) + t130 * t322) * L9;
      t486 = t319 * t274;
      t491 = L8 * t475;
      t493 = t17 * t326;
      t499 = t320 * (t114 * (-2 * t457 * t274 + 2 * t486) + 2 * t491 - t493 / 2 - 2 * L8 * L5 * t457);
      t502 = 2 * t130 * L8 * t322;
      t507 = L9 * theta6__dot;
      t510 = L6 * t176 - L9 / 4;
      t526 = 2 * t431 + theta1__dot;
      t531 = L9 * t320;
      t566 = L9 * L7 * t107;
      t585 = L9 * L7 * t25;
      t594 = t320 * t27;
      res__2 = 4 * t203 * theta1__dot * t200 + 2 * t316 * theta1__dot * t314 + t47 * (t55 * t341 + t35 * t356 + 8 * t363 * m6 * (t327 * t212 * t326 + t323 * t41) - 16 * t368 * L7 * t37 * t366 + 4 * t373 - 8 * t227 * t323) / 16 + t66 * (-t55 * t356 + t35 * t341 - 4 * t363 * (t212 * t347 - 4 * t368 * t343) * m6 - 16 * t327 * (-t225 / 2 + t228) * theta4__dot + 8 * t323 * t42 * t38) / 16 - t23 * t79 * (2 * t74 * (t327 * t240 + t320 * t248) * theta6__dot - 2 * t58 * t408 * t407 + (2 * t70 * t320 * t73 + t350) * theta4__dot) / 8 - t87 * t79 * (-2 * t74 * t424 * t327 * t58 - 2 * t327 * t424 * t84 + (4 * t431 + theta1__dot) * theta6__dot) / 16 + t449 * theta1__dot * (m3 * t441 + t444 / 4 - IX2 - IX3) * theta2__dot - t454 * t453 * t452 + t74 * (-8 * t58 * m6 * (t327 * t471 + t485 + t499 - t502) + t327 * (16 * t37 * t99 * t510 * t507 + 16 * L7 * (-t107 * t463 * t168 / 2 + t93 * t424 * t104)) - 8 * t37 * t526 * t130 * L9 * t222 + 16 * L7 * (-t25 * t531 * t333 + t130 * t93 * (t320 * (-t319 + t457) + t322))) / 16 + t58 * (-2 * t327 * (t70 * t510 * t507 + t149) * theta5__dot + t507 * t107 * (t70 * t526 * t130 + 0.3e1 / 0.2e1 * L7 * (0.4e1 / 0.3e1 * t431 + theta1__dot))) * m6 / 2 + t327 * (-8 * t37 * m6 * (t566 * t326 + t70 * t471) + 16 * theta5__dot * (-t28 + m6 * (t61 + t29 + t30) + t62 + t20) * theta4__dot) / 16 - t37 * (t70 * (t485 + t499 - t502) + t585 * t320 * t458) * m6 / 2 - t347 * t26 * t304 / 2 + t407 * t137 * t594 + 0.3e1 / 0.4e1 * t354 - t453 * t452 + c1;
      t606 = s ^ 2;
      t608 = L4 ^ 2;
      res__3 = t5 * (t9 * (m3 * t3 * (-0.1e1 * t443 + (-0.2e1 * s - 0.1e1 * L4) * L3 - 0.1e1 * s * L4 - 0.1e1 * t606 - 0.2500000000e0 * t608) + t3 * (IX2 + IX3 - 0.2500000000e0 * t444)) + (-0.4905e1 * L4 - 0.9810000000e1 * s - 0.981e1 * L3) * m3 - 0.4905000000e1 * L3 * m2) + (-0.2e1 * L3 * theta2__dot * s__dot - 0.1e1 * L4 * theta2__dot * s__dot - 0.2e1 * s * theta2__dot * s__dot) * m3;
      t638 = IZ6 * t3;
      t639 = 0.1250000000e0 * t638;
      t640 = m6 * t3;
      t641 = t17 * t640;
      t643 = -t639 + 0.3125000000e-1 * t641;
      t644 = t55 * t643;
      t646 = t3 * t26;
      t648 = 0.2500000000e0 * t646 * t27;
      t649 = 0.2500000000e0 * t29;
      t650 = 0.3125000000e-1 * t17;
      t651 = -t649 - t650;
      t654 = m6 * t3 * t651 + t639 + t648;
      t656 = L7 * L8;
      t661 = -0.5000000000e0 * t640 * t656 + 0.2500000000e0 * t646 * t288;
      t662 = t58 * t661;
      t665 = 0.2500000000e0 * IX5;
      t666 = 0.2500000000e0 * t62;
      t667 = t665 - t666;
      t669 = t23 * t644 + t55 * t654 + t662 - 0.2500000000e0 * t61 * t640 + t3 * t667;
      t672 = -t35 * t643;
      t675 = -t3 * t651;
      t683 = 0.3750000000e0 * t638;
      t684 = 0.9375000000e-1 * t641;
      t693 = -0.5000000000e0 * t3 * m5 * t91 - 0.5000000000e0 * t3 * m6 * t91;
      t695 = m6 * t114;
      t696 = t3 * t695;
      t697 = t696 * t102;
      t699 = L5 * L9;
      t700 = t640 * t699;
      t702 = 0.5000000000e0 * t697 + 0.5000000000e0 * t700;
      t704 = L5 * L8;
      t705 = t640 * t704;
      t707 = t696 * t274;
      t711 = L6 * L7;
      t712 = t640 * t711;
      t714 = t3 * m5;
      t715 = t714 * t711;
      t717 = -0.1e1 * t712 - 0.1e1 * t715;
      t719 = L5 * L7;
      t720 = t640 * t719;
      t722 = t714 * t719;
      t733 = t3 * t137;
      t740 = -0.1e1 * t640 * t180 - 0.1e1 * t714 * t180;
      t755 = t66 * t669 + t47 * (t23 * t672 + t35 * (m6 * t675 - t639 - t648) - t37 * t661) + t23 * (-t683 + t684) + t95 * t693 + t70 * (t58 * (t25 * t702 - 0.1e1 * t705 - 0.1e1 * t707) + t114 * t717 - 0.1e1 * t720 - 0.1e1 * t722) + t662 + t37 * t74 * (t130 * t3 * t213 - t25 * t702) - 0.5000000000e0 * t733 * t304 + t648 + t114 * t740 + m6 * t3 * (-0.5000000000e0 * t184 + t650 - t649 - 0.2500000000e0 * t61) + t3 * (-t666 + t184 * (-0.2000000000e1 * m4 - 0.5000000000e0 * m5) + 0.5000000000e0 * IX4 - 0.1250000000e0 * IZ6 + t665);
      t757 = IZ6 * t322;
      t758 = 0.5000000000e0 * t757;
      t759 = t18 * t322;
      t761 = -t758 + 0.1250000000e0 * t759;
      t762 = t327 * t761;
      t764 = theta5__dot * t17;
      t765 = t764 * t332;
      t768 = theta4__dot * IZ6 * theta5__dot;
      t769 = 0.5000000000e0 * t768;
      t770 = -0.1250000000e0 * t765 + t769;
      t774 = t18 * t326;
      t775 = 0.1250000000e0 * t774;
      t776 = IZ6 * t326;
      t777 = 0.5000000000e0 * t776;
      t778 = -t775 + t777;
      t780 = t25 * t27;
      t781 = theta1__dot * m6;
      t782 = theta5__dot * t781;
      t783 = t782 * t780;
      t784 = 0.1250000000e0 * t764;
      t785 = theta5__dot * t29;
      t793 = 0.5000000000e0 * t333 * t109;
      t796 = IZ6 * t347;
      t798 = t18 * t347;
      t806 = t366 * t780;
      t815 = t87 * t327 * (0.5000000000e0 * t796 - 0.1250000000e0 * t798) + 0.5000000000e0 * t372 * t327 * t27 - 0.1e1 * t806 + m6 * (theta5__dot * theta4__dot * t29 + 0.1250000000e0 * theta5__dot * theta4__dot * t17) - t769;
      t817 = L8 * t57;
      t819 = 0.2000000000e1 * t817 * t322;
      t820 = t782 * t585;
      t826 = 0.1e1 * t62;
      t832 = t58 * t288;
      t833 = t326 * t137;
      t842 = 0.2000000000e1 * L8 * theta5__dot * m6 * t148 - 0.1e1 * t366 * t585;
      t855 = 0.1e1 * t783;
      t875 = m5 * theta4__dot;
      t883 = 0.2500000000e0 * t798;
      t884 = t796 - t883;
      t885 = t320 * t884;
      t887 = 0.2500000000e0 * t18 * t407;
      t888 = IZ6 * t407;
      t889 = t885 - t887 + t888;
      t898 = -0.1250000000e0 * t641 + 0.5000000000e0 * t638;
      t904 = -t37 * t74 * t898 + t70 * t58 * t898;
      t906 = 0.2500000000e0 * t759;
      t907 = t757 - t906;
      t910 = 0.5000000000e0 * IZ6 * t457;
      t911 = m6 * t457;
      t913 = 0.1250000000e0 * t17 * t911;
      t918 = -0.1e1 * t757 + t906;
      t919 = t320 * t918;
      t927 = L9 * t213;
      t928 = t927 * t107 * t3;
      t929 = 0.5000000000e0 * t928;
      t930 = m6 * t100;
      t932 = t3 * t930 * t274;
      t934 = t646 * t304;
      t935 = 0.5000000000e0 * t934;
      t938 = t733 * t288;
      t954 = t114 * t274;
      t956 = 0.2000000000e1 * t782 * t954;
      t957 = m6 * L5;
      t960 = 0.2000000000e1 * L8 * t957 * t322;
      t963 = t107 * t347;
      t967 = L6 * m6;
      t968 = L7 * t967;
      t971 = m5 * t711;
      t974 = -0.2000000000e1 * t968 * t322 - 0.2000000000e1 * t971 * t322;
      t976 = m5 * t719;
      t979 = m6 * t719;
      t984 = t782 * t594;
      t985 = t114 * t102;
      t986 = t333 * t985;
      t988 = 0.5000000000e0 * t458 * L8;
      t990 = 0.5000000000e0 * t457 * L8;
      t991 = theta6__dot * t117;
      t997 = 0.4905000000e1 * t168;
      t998 = t782 * t304;
      t1003 = t457 * L6;
      t1005 = 0.5000000000e0 * t458 * L6 + 0.5000000000e0 * t1003;
      t1012 = 0.9810000000e1 * t213;
      t1013 = -0.2000000000e1 * t782 * t105 - t1012;
      t1016 = t457 * t930 * t274;
      t1023 = t320 * theta1__dot + theta5__dot;
      t1033 = t320 * t17;
      t1034 = theta6__dot * t781;
      t1045 = t107 * t1023 * t168 * t100 * theta6__dot * L6 + t25 * (-0.1e1 * t366 * t985 - 0.1e1 * t366 * t699) - 0.2500000000e0 * t1034 * t1033 + 0.2000000000e1 * t366 * t954 + m6 * (0.2000000000e1 * t343 * t704 - 0.2500000000e0 * theta6__dot * t764);
      t1048 = t782 * t320 * t288;
      t1052 = 0.5000000000e0 * t457 * L7;
      t1059 = L7 * m5;
      t1060 = 0.9810000000e1 * t1059;
      t1061 = 0.9810000000e1 * t57;
      t1079 = L9 * t957;
      t1094 = 0.9810000000e1 * t967 + 0.9810000000e1 * L6 * m5;
      t1097 = 0.1e1 * t29;
      t1114 = 0.1e1 * t984;
      t1135 = theta5__dot * t875;
      res__4 = t203 * t755 + t66 * (t23 * (t35 * t770 + t55 * t762) + t55 * (t87 * t778 + t327 * (t783 + m6 * theta1__dot * (-t784 - 0.1e1 * t785) + t758) + t793) + t35 * t815 + t327 * (t58 * (-t819 + t820) + 0.5000000000e0 * t373 - 0.1e1 * t63 * t322 + theta1__dot * theta5__dot * (IX5 - t826)) + 0.5000000000e0 * t833 * t832 + t37 * t842) + t47 * (t23 * (-t35 * t327 * t761 + t55 * t770) + t55 * t815 + t35 * (-t87 * t778 + t327 * (-t855 + m6 * theta1__dot * (t785 + t784) - t758) - t793) + t327 * (0.5000000000e0 * t372 * t832 + t37 * (-0.1e1 * t820 + t819)) + t58 * t842 - 0.5000000000e0 * t833 * t224 + theta5__dot * t61 * t332 + theta5__dot * (t61 * t875 - 0.1e1 * theta4__dot * IX5)) + t23 * (t70 * t37 * t889 + t58 * t74 * t889 + t762) + t87 * (t316 * t904 + t70 * t58 * (t320 * t907 + t910 - t913) + t37 * t74 * (t919 - t910 + t913) - t777 + t775) + t316 * (t257 * t693 + t70 * (t58 * (t929 - 0.1e1 * t932 + t935) + 0.5000000000e0 * t938 + t100 * t717) + t37 * t74 * (-t929 + t932 - t935) + t107 * t702 + t100 * t740) + t70 * (t327 * (t58 * (t25 * t130 * m6 * L9 * t322 - t956 - t960) + t37 * t130 * t168 * t963 + t114 * t974 - 0.2000000000e1 * t976 * t322 - 0.2000000000e1 * t979 * t322) + t58 * (t107 * (t984 + t986 + m6 * L9 * (-t988 + t990 + t991)) + t25 * (t320 * (t997 + t998) + t930 * L9 * t1005) + t320 * t1013 - 0.1e1 * t1016) + t37 * t1045 + t107 * (t1048 + m6 * L9 * (-0.5000000000e0 * t458 * L7 + t1052)) + t320 * (t100 * t974 - t1060 - t1061) + t100 * (-0.1e1 * t1059 * t1003 - 0.1e1 * t711 * t911)) + t327 * (t58 * (t74 * t130 * t168 * t963 - t819 + t820) + t37 * (t74 * (t25 * (-0.1e1 * t782 * t985 - 0.1e1 * t1079 * t322) + t960 + t956) + 0.5000000000e0 * t1034 * t566) - 0.4905000000e1 * t138 + t783 + t100 * t1094 + m6 * theta1__dot * (-t784 + theta5__dot * (-0.1e1 * t61 - t1097)) + theta1__dot * theta5__dot * (-0.500000000e0 * IZ6 - t826)) + t58 * (t74 * t1045 + 0.5000000000e0 * t333 * t566) + t37 * t74 * (t107 * (-t1114 - 0.1e1 * t986 + m6 * L9 * (t988 - t990 - 0.1e1 * t991)) + t25 * (t320 * (-0.1e1 * t998 - t997) - t930 * L9 * t1005) - t320 * t1013 + t1016) + t74 * (t114 * (0.2000000000e1 * t1135 * t711 + 0.2000000000e1 * t366 * t711) + 0.2e1 * t1135 * t719 + 0.2e1 * t366 * t719) + t107 * (-0.5000000000e0 * t458 * t695 * t102 + m6 * L9 * (-0.5000000000e0 * L5 * t458 + 0.5000000000e0 * t476)) - 0.1e1 * t326 * t26 * t304 + t320 * (-t114 * t1094 - 0.9810000000e1 * t957 + (-0.9810e1 * m5 - 0.1962e2 * m4) * L5);
      t1179 = 0.2500000000e0 * IZ6 * t319;
      t1180 = m6 * (-0.3125000000e-1 * t3 * t17 + 0.6250000000e-1 * t17 * t319) + t639 - t1179;
      t1182 = theta1__dot * theta4__dot;
      t1183 = IZ6 * t1182;
      t1184 = 0.5000000000e0 * t1183;
      t1185 = t18 * t1182;
      t1186 = 0.1250000000e0 * t1185;
      t1187 = -t1184 + t1186;
      t1188 = t327 * t1187;
      t1189 = t35 * t1188;
      t1205 = t25 * m6 * (-0.2500000000e0 * t3 * t27 + 0.5000000000e0 * t319 * t27) + m6 * (t675 + t319 * (-0.5000000000e0 * t29 - 0.6250000000e-1 * t17)) - t639 + t1179;
      t1207 = theta4__dot * t781;
      t1208 = t1207 * t780;
      t1209 = 0.1250000000e0 * t17;
      t1228 = m6 * (-0.2500000000e0 * t3 * t288 + 0.5000000000e0 * t319 * t288);
      t1235 = m6 * (-0.1e1 * t319 * t656 + 0.5000000000e0 * t3 * t656);
      t1279 = 0.2000000000e1 * t817 * t1182;
      t1289 = t25 * t37;
      t1296 = 0.2500000000e0 * t700 + 0.2500000000e0 * t697;
      t1298 = 0.5000000000e0 * t707;
      t1299 = 0.5000000000e0 * t705;
      t1302 = 0.5000000000e0 * t715;
      t1304 = -t1302 - 0.5000000000e0 * t712;
      t1307 = 0.5000000000e0 * t722;
      t1326 = -0.2500000000e0 * t774 + t776;
      t1335 = 0.6250000000e-1 * t641 - 0.2500000000e0 * t638;
      t1342 = 0.2500000000e0 * t928;
      t1343 = 0.5000000000e0 * t932;
      t1344 = 0.2500000000e0 * t934;
      t1353 = t100 * t37 * t102;
      t1354 = t25 * t70;
      t1355 = t640 * t1354;
      t1363 = 0.2500000000e0 * t1185;
      t1370 = t320 * (-0.1e1 * t1183 + t1363);
      t1379 = t1207 * t985;
      t1380 = theta6__dot * t27;
      t1382 = theta4__dot * t699;
      t1390 = 0.2000000000e1 * t1207 * t954;
      t1395 = -0.2000000000e1 * theta4__dot * t704 + 0.2500000000e0 * theta6__dot * t17;
      t1402 = 0.1e1 * t1379;
      t1409 = m6 * t288;
      t1421 = t70 * t288;
      t1425 = m6 * t70;
      t1437 = 0.2000000000e1 * t968 * t1182 + 0.2000000000e1 * t971 * t1182;
      t1469 = t114 * m6 * (0.5000000000e0 * t319 * t102 + 0.2500000000e0 * t3 * t102) + m6 * (0.2500000000e0 * t3 * t699 - 0.1e1 * t326 * t27 + 0.5000000000e0 * t319 * t699);
      t1476 = t114 * m6 * (-0.1e1 * t486 - 0.5000000000e0 * t3 * t274);
      t1482 = m6 * (-0.1e1 * t491 - 0.5000000000e0 * t3 * t704 + 0.2500000000e0 * t493);
      t1486 = t1182 * m6 * t320;
      t1493 = 0.2000000000e1 * t1207 * t105;
      t1494 = t1207 * t109;
      t1504 = t319 * L6;
      t1511 = L5 * t3;
      res__5 = t47 * (t316 * t669 + t23 * (t55 * t1180 + t1189) + t55 * t1205 + t35 * t327 * (t1208 + t781 * theta4__dot * (-t1209 - t1097) + t1184) + t327 * (-0.2000000000e1 * t1207 * t37 * t656 + t1182 * t26 * t224) + t58 * (t25 * t1228 + t1235) + m6 * (-0.5000000000e0 * t61 * t319 + 0.2500000000e0 * t61 * t3) - t3 * t667 + t319 * (-0.5000000000e0 * t62 + 0.5000000000e0 * IX5)) + t66 * (t316 * (t23 * t35 * t643 + t35 * t654 - 0.5000000000e0 * t3 * t38 * t656 + 0.2500000000e0 * t646 * t224) + t23 * (-t55 * t327 * t1187 + t35 * t1180) + t55 * t327 * (-0.1e1 * t1208 + t781 * theta4__dot * (t29 + t1209) - t1184) + t35 * t1205 + t327 * (t58 * (-0.1e1 * t1207 * t585 + t1279) + t63 * t1182 + theta1__dot * theta4__dot * (-0.1e1 * IX5 + t62)) + t1289 * t1228 + t37 * t1235) + t316 * (t74 * (t58 * (t25 * t1296 - t1298 - t1299) + t114 * t1304 - 0.5000000000e0 * t720 - t1307) + t1289 * t70 * t1296 + t37 * t70 * (-t1298 - t1299)) + t23 * (t327 * (t74 * t37 * t884 + t58 * t70 * (-0.1e1 * t796 + t883) + t1184 - t1186) + t74 * t58 * t1326 + t37 * t70 * t1326) + t203 * (t87 * (t37 * t70 * t1335 + t74 * t58 * t1335) + t74 * (t58 * (-t1342 + t1343 - t1344) - t100 * t1304 - 0.2500000000e0 * t938) - 0.2500000000e0 * t1355 * t1353 + t37 * t70 * (-t1342 + t1343)) + t87 * (t74 * t37 * t320 * (t1183 - t1363) + t58 * t70 * t1370 + t885 - t887 + t888) + t327 * (t74 * (t58 * (t1012 - 0.4905000000e1 * m6 * t39) + t25 * t37 * (t1379 + m6 * theta1__dot * (-0.1e1 * t1380 + t1382)) + t37 * (m6 * theta1__dot * t1395 - t1390) + t1060 + t1061) + t58 * (t25 * (t70 * (-t1402 + m6 * theta1__dot * (t1380 - 0.1e1 * t1382)) - 0.1e1 * t1409 * t1182) + t70 * (-m6 * theta1__dot * t1395 + t1390) + t1279) + t25 * (t1034 * t1421 - 0.1e1 * t927 * t1182 - 0.4905000000e1 * t1425 * t223) + 0.9810000000e1 * t1425 * t37 * L8 + t70 * (t114 * t1437 + 0.2000000000e1 * t976 * t1182 + 0.2000000000e1 * t979 * t1182) + t781 * theta4__dot * (t61 + t29 + t1209) + theta1__dot * theta4__dot * (t62 + 0.5000000000e0 * IZ6)) + t74 * (t58 * (t25 * t1469 + t1476 + t1482) + t25 * (t1486 * t1353 - 0.1e1 * t1409 * t326) + t37 * t320 * (-t1493 + t1494) + t114 * (m6 * (-0.1e1 * t319 * t711 - 0.5000000000e0 * t3 * t711) - t1302 - 0.1e1 * t1059 * t1504) + m6 * (-0.1e1 * t319 * t719 - 0.5000000000e0 * L7 * t1511) - 0.1e1 * t1059 * t475 - t1307) + t58 * (-0.1e1 * t1207 * t1354 * t320 * t100 * t102 + t70 * t320 * (t1493 - 0.1e1 * t1494) + t1023 * t168 * L7 * t236) + t25 * t37 * (t70 * t1469 - 0.5000000000e0 * m6 * t458 * t288) + t37 * t70 * (t1476 + t1482) + t70 * t320 * (t100 * t1437 - 0.1e1 * t1207 * t566) + t372 * t594 + m6 * t407 * t109 + c5;
      t1557 = 0.6250000000e-1 * t319;
      t1558 = 0.3125000000e-1 * t3;
      t1559 = t1557 - t1558;
      t1594 = -t1296;
      t1596 = t3 * L7;
      t1609 = L8 * t3;
      t1612 = 0.2500000000e0 * t319 * L8;
      t1613 = 0.1250000000e0 * t1609 - t1612;
      t1619 = t1182 * t137;
      t1624 = 0.2500000000e0 * t319 * L7;
      t1625 = 0.1250000000e0 * t1596 - t1624;
      t1629 = t1182 * m6 * t327;
      t1640 = 0.2500000000e0 * t765;
      t1641 = t1370 - 0.1e1 * t768 + t1640;
      t1696 = 0.4905000000e1 * m6 * t327 * L9;
      t1700 = -0.5000000000e0 * t1504 - 0.2500000000e0 * t3 * L6;
      t1702 = t114 * L9;
      t1706 = -0.2500000000e0 * t1511 - 0.5000000000e0 * t475;
      t1717 = t327 * (-t1402 - 0.1e1 * t1079 * t1182) - 0.1e1 * t1486 * t304;
      t1728 = -m6 * t1700;
      t1751 = -t1640 + t806 + 0.2500000000e0 * t1207 * t1033;
      res__6 = t87 * (t316 * (t47 * t672 + t66 * t644 - t683 + t684) + t66 * (t55 * (t17 * m6 * t1559 - t1179 + t639) + t1189) + t47 * (t55 * t1188 + t35 * (-t17 * m6 * t1559 + t1179 - t639)) + t919 + t17 * m6 * (t1558 + 0.1250000000e0 * t457 - t1557) - t639 + t1179 - t910) + t316 * (t66 * (-0.1250000000e0 * t733 * t55 * t27 - 0.1250000000e0 * t733 * t832) + t47 * (0.1250000000e0 * t733 * t35 * t27 + 0.1250000000e0 * t733 * t224) + t107 * (t58 * (t70 * t1594 - 0.1250000000e0 * t168 * t1596) + t37 * t74 * t1296 - 0.1250000000e0 * t27 * t640) - t1344) + t66 * (t55 * t212 * m6 * t1613 - 0.5000000000e0 * t1619 * t327 * t35 * t27 + t107 * (t237 * m6 * t1625 - 0.5000000000e0 * t1629 * t224)) + t23 * (t203 * t904 + t58 * (t70 * t327 * t907 + t74 * t1641) + t37 * (t74 * t327 * t918 + t70 * t1641)) + t47 * (-0.5000000000e0 * t1619 * t327 * t55 * t27 - t35 * t212 * m6 * t1613 + t107 * (-0.5000000000e0 * t1629 * t832 - t223 * m6 * t1625)) + t203 * (t107 * (0.2500000000e0 * t640 * t70 * t100 * t58 * t102 - 0.2500000000e0 * t640 * t74 * t100 * t37 * t102) - 0.2500000000e0 * t1355 * t58 * t27 + 0.2500000000e0 * t640 * t25 * t74 * t37 * t27 - 0.2500000000e0 * t646 * t1421 + t25 * t1594) + t107 * (t58 * (t70 * (L9 * m6 * t1706 + t1702 * m6 * t1700 + t1696) + t74 * t1717 - 0.1e1 * t1048 + L9 * m6 * (-t1624 - 0.3750000000e0 * t1596 - t1052)) + t37 * (t70 * t1717 + t74 * (-L9 * m6 * t1706 + t1702 * t1728 - t1696) - 0.5000000000e0 * t1207 * t327 * t288) - t1114 + L9 * m6 * (-t1612 - t990 - 0.3750000000e0 * t1609)) + t58 * (t70 * t327 * (-t855 + t906) + t74 * t1751) + t37 * (t70 * t1751 + t74 * t327 * (-t906 + t783)) - 0.1e1 * t322 * t26 * t408 * t288 + t343 * t26 * t74 * t288 + t25 * (-0.4905000000e1 * m6 * t531 + t303 * t1728) + c6;
      
      % store on output
      res__gfun = zeros(6,1);
      res__gfun(1) = res__1;
      res__gfun(2) = res__2;
      res__gfun(3) = res__3;
      res__gfun(4) = res__4;
      res__gfun(5) = res__5;
      res__gfun(6) = res__6;
    end

    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % function rhs = gforce_q( self, t, pos, vel )
    %   theta1 = pos(1); x2 = pos(2);
    %   omega1 = vel(1); u2 = vel(2);
    %   L     = self.L;
    %   x30   = self.x30;
    %   kappa = self.kappa;
    %   c     = self.c;
    %   pos_D = self.positive_part_D( (L + x2 - x30) * kappa - u2 * c);
    %   rhs   = pos_D * [ 0, 0; 0, -kappa ];
    % end
    % % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % function rhs = gforce_v( self, t, pos, vel )
    %   theta1 = pos(1); x2 = pos(2);
    %   omega1 = vel(1); u2 = vel(2);
    %   L     = self.L;
    %   x30   = self.x30;
    %   kappa = self.kappa;
    %   c     = self.c;
    %   pos_D = self.positive_part_D( (L + x2 - x30) * kappa - u2 * c);
    %   rhs   = pos_D * [ 0, 0; 0, c ];
    % end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % d^2 Phi(t,q) / dt^2 = Phi_q(t,q) v' - B(t,q,v)
    function res__B = b( self, t, pos, vel )
      vars__=[pos, vel];

      L1=self.L1;
      L2=self.L2;
      L3=self.L3;
      L4=self.L4;
      L5=self.L5;
      L6=self.L6;
      L7=self.L7;
      L8=self.L8;
      L9=self.L9;
      L10=self.L10;
      phi1 = self.phi1; 
      phi2 = self.phi2;
      m1 = self.m1;
      m2 = self.m2;
      m3 = self.m3;
      m4 = self.m4;
      m5 = self.m5;
      m6 = self.m6;
      IZ1 = self.IZ1;
      IX2 = self.IX2;
      IX3 = self.IX3;
      IX4 = self.IX4;
      IX5 = self.IX5;
      IZ6 = self.IZ6;

      % extract states
      s = vars__(1);
      theta1 = vars__(2);
      theta2 = vars__(3);
      theta4 = vars__(4);
      theta5 = vars__(5);
      theta6 = vars__(6);
      s__dot = vars__(7);
      theta1__dot = vars__(8);
      theta2__dot = vars__(9);
      theta4__dot = vars__(10);
      theta5__dot = vars__(11);
      theta6__dot = vars__(12);
      % evaluate function
      t1 = theta2__dot ^ 2;
      t2 = cos(theta2);
      t4 = L3 + L4 + s;
      t6 = sin(theta2);
      t10 = theta4__dot ^ 2;
      t12 = cos(theta4);
      res__1 = t12 * t10 * L5 - 2 * s__dot * t6 * theta2__dot - t4 * t2 * t1;
      t19 = sin(theta4);
      res__2 = L5 * t19 * t10 + 2 * t2 * theta2__dot * s__dot - t6 * t1 * t4;
      
      % store on output
      res__B = zeros(2,1);
      res__B(1) = res__1;
      res__B(2) = res__2;
      end

    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % function rhs = b_q( self, t, pos, vel )
    %   theta1 = pos(1); x2 = pos(2);
    %   omega1 = vel(1); u2 = vel(2);
    %   R      = self.R;
    %   alpha  = self.alpha;
    %   rhs    = [sin(alpha+theta1)*omega1^2*R;0];
    % end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % function rhs = b_v( self, t, pos, vel )
    %   theta1 = pos(1); x2 = pos(2);
    %   omega1 = vel(1); u2 = vel(2);
    %   R      = self.R;
    %   alpha  = self.alpha;
    %   rhs    = [-2*cos(alpha+theta1)*omega1*R;0];
    % end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % function J = W_q( self, t, pos, v_dot )
    %   J = [R*cos(alpha+theta1)*v1, 0];
    % end
    % % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % function res = T( self, t )
    %   res = 1;
    % end
    % % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % function res = positive_part( self, x )
    %   res = max(x,0);
    % end
    % % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % function res = positive_part_D( self, x )
    %   if x > 0
    %     res = 1;
    %   else
    %     res = 0;
    %   end;
    % end
    % % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % function x2 = theta_to_x2( self, theta1 )
    %   alpha = self.alpha;
    %   R     = self.R;
    %   H     = self.H;
    %   x2    = (sin(alpha) * H + cos(alpha + theta1) * R)/cos(alpha);
    % end
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % function plot( self, pos )
    %   theta1 = pos(1);
    %   x2     = pos(2);
    %   R      = self.R;
    %   x30    = self.x30;
    %   H      = self.H;
    %   alpha  = self.alpha;
    %   ell    = R;
    %   xx1    = x2+ell*sin(alpha);
    %   yy1    = H+ell*cos(alpha);
    %   xx2    = x2-(ell+2*H)*sin(alpha);
    %   yy2    = H-(ell+2*H)*cos(alpha);
    %   hh     = R/4;

    %   hold off;
    %   drawAxes(2,0.25,0.5,0,0);
    %   hold on;
    %   fillRect( 'blue', xx1-xx2+2*hh, yy1-yy2+2*hh, xx2-hh, yy2-hh );
    %   fillRect( 'blue', x30, 2*hh, x2, H-hh );
    %   drawLine( xx1, yy1, xx2, yy2, 'LineWidth', 4, 'Color', 'white' );
    %   %drawLine( x2, H, x2+x30, H, 'LineWidth', 4, 'Color', 'blue' );
    %   drawLine( 0, 0, R*cos(theta1), R*sin(theta1), 'LineWidth', 2, 'Color', 'red' );
    %   drawCircle( 0, 0, R, 'LineWidth', 2, 'Color', 'k' ); % 0, 180*theta1/pi,
    %   drawCOG( R/10, 0, 0 );
    %   drawCOG( R/10, R*cos(theta1), R*sin(theta1) );
    %   drawCOG( R/10, x2, H );
    %   xlim([-0.2,0.8]);
    %   ylim([-0.2,0.2]);
    % end

        %
    %  Abstract functions defining an index-3 DAE with some derivatives
    %
    %  q' = v
    %  M(t,p) v' + Phi_p(t,q)^T lambda = gforce( t, q, v )
    %  Phi(t,q) = 0
    %
    %  d Phi(t,q) / dt     = A(t,q,v)
    %  d^2 Phi(t,q) / dt^2 = Phi_q(t,q) v - b(t,q,v)
    %
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % return function \partial Phi_q(t,q)*v / \partial q
    PhiV_q( self, t, q, v_dot )
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % return function \partial Phi_q(t,q)^T*lambda / \partial q
    PhiL_q( self, t, q, lambda )
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % return function \partial ( M(t,q) v_dot ) / \partial q
    W_q( self, t, q, v_dot )
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % return function \partial f( t, q, v ) / \partial q
    gforce_q( self, t, q, v )
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % return function \partial f( t, q, v ) / \partial v
    gforce_v( self, t, q, v )
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % return function \partial b( t, q, v ) / \partial q
    b_q( self, t, q, v )
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    % return function \partial b( t, q, v ) / \partial q
    b_v( self, t, q, v )
    % - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  end
end
