%% test_airplane.m Testing reaching a point or following a time-varying point
dt = 0.01;
T0 = 0.;
T1 = 5.; %5

% Defining an airplane with Bella's dynamics system and satisfying contraints and objective
sys = Kirby_Plane(dt, 0.1);
[x, u, y] = sys.signals();

% Reaching a fixed point
state_initial = [33.040458; -97.279331; 16.764; 0; 0; -2.79; 0; 0; 0; -37.21; -102.38; 0];
state_final = [33.040022; -97.280124; 16.764; 0; 0];

%initial_string = '';
%for i = 1:length(state_initial)
%    initial_string = sprintf('%s %6.6f;', initial_string, state_initial(i));
%end

%objective_string = '';
%for i = 1:length(state_final)
%    objective_string = sprintf('%s %6.6f;', objective_string, state_final(i));
%end

%initial_string = sprintf('x(0) = [%s]', initial_string);
%objective_string = sprintf('norm(x(t, 1:%d) - [%s])^2', length(final_string), final_string);

%sys.set_objective(objective_string);
%sys.add_constraint(P(initial_string));

sys.add_constraint(P('x(0) == state_initial'));
sys.set_objective('norm(x(t, 1:5)-state_final)^2');

sys.add_constraint(P('u(0)==[0; 0; 0; 1]'));
sys.run_closed_loop(5, T0, T1)



