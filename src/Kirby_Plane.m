classdef Kirby_Plane < System
    properties
        size
    end
    methods
        function self = Kirby_Plane(dt, size)
            self@System(dt);
            self.size = size;
            
            dyn = Dynamics(12, 4);
            [x, u] = dyn.symbols();
            % x(1:3): lat, lon and alt (meters).
            % x(3:6): phi, theta, psi
            % x(6:9): p, q, r
            % x(9:12): Veast, Vnorth, Vdown in ft/sec 
            % u(1:4): ele_cmd, ail_cmd, rud_cmd, thtl_cmd
            
            lat = x(1);
            lon = x(2);
            alt = x(3);
            phi = x(4);
            theta = x(5);
            psi = x(6);
            p = x(7);
            q = x(8);
            r = x(9);
            Veast = x(10);
            Vnorth = x(11);
            Vdown = x(12);
            
            ele_cmd = u(1);
            ail_cmd = u(2);
            rud_cmd = u(3);
            thtl_cmd = u(4);
            control_cmd = [ele_cmd; ail_cmd; rud_cmd; thtl_cmd];

            edge_540_constants = load('data_AAA_Edge540');
            
            gamma = atan(-Vdown/sqrt(Vnorth^2+Veast^2));
            alpha_prime = theta-gamma;
            alpha = atan(cos(phi)*tan(alpha_prime));
            beta = asin(sin(phi)*sin(alpha_prime));
            
            % Body Velocity : Converting GPS velocity into the body frame velocity
            h11 = cos(theta)*cos(psi);
            h12 = -cos(phi)*sin(psi) + sin(phi)*sin(theta)*cos(psi);
            h13 = sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi);
            h21 = cos(theta)*sin(psi);
            h22 = cos(phi)*cos(psi) + sin(phi)*sin(theta)*sin(psi);
            h23 = -sin(phi)*cos(psi) + cos(phi)*sin(phi)*sin(psi);
            h31 = -sin(theta);
            h32 = sin(phi)*cos(theta);
            h33 = cos(phi)*cos(theta);
            HB2I = [h11,h12,h13; h21,h22,h23; h31,h32,h33];
            
            V_B = HB2I'*[Veast; Vnorth; Vdown]; %Velocity was converted from inertial to body frame
            
            uu = V_B(1);
            v = V_B(2);
            w = V_B(3);
            
            Vtotal = sqrt(uu*uu + v*v + w*w);
            % beta = asin(v/Vtotal);
            % alpha = atan(w/u);
            
            %[CD, CL, Cm, Cl, Cy, Cn] = controlderivative2(alpha, beta, control_cmd, edge_540_constants);
            
            % Calculating Control Derivative 
            control = [u(1); u(2); u(3); u(4)];

            %control = u(1:4);
            control = control';
            d_e = control(:,1);
            d_a = control(:,2);
            d_r = control(:,3);

            long_coef = [edge_540_constants.CD0          , edge_540_constants.CDAlpha, edge_540_constants.CDHorTailIncidence, edge_540_constants.CDdeltaElevator;...
                         edge_540_constants.CLzeroAirplane, edge_540_constants.CLAlpha, edge_540_constants.CLHorTailIncidence, edge_540_constants.CLdeltaElevator;...
                         edge_540_constants.CmzeroAirplane, edge_540_constants.CmAlpha, edge_540_constants.CmHorTailIncidence, edge_540_constants.CmdeltaElevator]*[1;alpha;0;d_e];
            CD = long_coef(1,:);
            CL = long_coef(2,:);
            Cm = long_coef(3,:);
    
            lat_coef=[edge_540_constants.ClBeta, edge_540_constants.CldeltaAileron, edge_540_constants.CldeltaRudder;...
                      edge_540_constants.CyBeta, edge_540_constants.CydeltaAileron, edge_540_constants.CydeltaRudder;...
                      edge_540_constants.CnBeta, edge_540_constants.CndeltaAileron, edge_540_constants.CndeltaRudder]*[beta;d_a;d_r];
            Cl = lat_coef(1,:);
            Cy = lat_coef(2,:);
            Cn = lat_coef(3,:);
            % Done with the Control Derivative
            
            q_bar = 0.002275*norm(Vtotal)^2*0.5;
            F_Ax = -CD*q_bar*edge_540_constants.S;
            F_Az = -CL*q_bar*edge_540_constants.S;
            M_A = Cm*q_bar*edge_540_constants.S*edge_540_constants.c;
            
            Thrust = control_cmd(4,:)*300*550*(1/Vtotal);
            phi_T = 0;   % x_B was parallel with the thrust line.
            dT = 0.8577; % ft, distance between xB and Thrust Line
            % For Edge 540
            F_Tx = Thrust*cos(phi_T+edge_540_constants.Alpha1*pi/180);
            F_Tz = -Thrust*sin(phi_T+edge_540_constants.Alpha1*pi/180);
            M_T = -Thrust*dT;
            
            L_A = Cl*q_bar*edge_540_constants.S*edge_540_constants.b;
            F_Ay = Cy*q_bar*edge_540_constants.S;
            N_A = Cn*q_bar*edge_540_constants.S*edge_540_constants.b;
            
            y_T = 0; % Aligned at the nose
            L_T = -Thrust*y_T*edge_540_constants.Alpha1*pi/180;
            F_Ty = 0;
            N_T = -Thrust*y_T;
            g = 32.2;
            
            %Steady state value forces and moments
            F_x = - (edge_540_constants.CD1+edge_540_constants.CTx1)*q_bar*edge_540_constants.S ;
            F_y = 0;
            F_z = -edge_540_constants.CL1*q_bar*edge_540_constants.S;
            L_1 = 0;
            M_1 = (edge_540_constants.Cm1+edge_540_constants.CmT1)*q_bar*edge_540_constants.S*edge_540_constants.c;
            N_1 = 0;
            
            %Coordinate Transformatoin from Stability to Body Coordinate
            HS2B = [cos(alpha)*cos(beta), -cos(alpha)*sin(beta), -sin(alpha);...
                sin(beta), cos(beta), 0;...
                sin(alpha)*cos(beta), -sin(alpha)*sin(beta), cos(alpha)];
            
            F_Body = HS2B*([F_Ax+F_Tx; F_Ay+F_Ty; F_Az+F_Tz]+[F_x;F_y;F_z]);
            M_Body = HS2B*([L_A+L_T; M_A+M_T; N_A+N_T]+[L_1; M_1; N_1]);
            
            m = 1350/g; % AAA
            
            % Define the moment of inertia in the body coordinates
            I_B = [edge_540_constants.Ixxb, 0   , -edge_540_constants.Ixzb;...
                0  , edge_540_constants.Iyyb,    0;...
                -edge_540_constants.Ixzb, 0   ,  edge_540_constants.Izzb];
            
            
            % Using the current state, calculate linear, angular acceleration and kinematic equation
            w_telda = [   0  , -r, q;...
                r,    0   ,-p;...
                -q,  p, 0];
            
            LE2B = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);...
                0,    cos(phi)             ,        -sin(phi);...
                0, sin(phi)*sec(theta), cos(phi)*sec(theta)];
            
            omega_dot = inv(I_B)*(M_Body-w_telda*I_B*[p;q;r]); %#ok<MINV>
            Euler_dot = LE2B*[p;q;r];
            VB_dot = (1/m)*F_Body + g*[-sin(theta); cos(theta)*sin(phi); cos(theta)*cos(phi)] - w_telda*[uu; v; w];
            
            VI_dot = HB2I*VB_dot;
            
            phi_dot = Euler_dot(1);
            theta_dot = Euler_dot(2);
            psi_dot = Euler_dot(3);
            p_dot = omega_dot(1);
            q_dot = omega_dot(2);
            r_dot = omega_dot(3);
            ve_dot = VI_dot(1);
            vn_dot = VI_dot(2);
            vh_dot = VI_dot(3);
            
            delta_north = lat + km2deg(1);
            delta_east = lon + km2deg(1)/cos(deg2rad(lat));
            %[delta_north, ~] = myReckon(lat, lon, km2deg(1), 0);
            %[~, delta_east] = myReckon(lat, lon, km2deg(1), 90);
            
            lat_dot = Vnorth*(delta_north - lat)/1000;
            lon_dot = Veast*(delta_east - lon)/1000;
            alt_dot = -Vdown * 0.3048;
            
            dyn.set_f([lat_dot; lon_dot; alt_dot; phi_dot; theta_dot; psi_dot; p_dot; q_dot; r_dot; ve_dot; vn_dot; vh_dot]);
            dyn.set_g(x(1));
            self.set_dynamics(dyn);
            
        end
    end
    
end