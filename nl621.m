% Definitions for Map automaton G_M
E1 = ['n', 's', 'e', 'w'];  % set of valid events
X1 = ['r1'; 'r2'; 'r3'; 'r4'; 'r5'; 'r6'; 'r7'];  % set of valid states

% Transitions for G_M in the form [starting_state, next_state, event]
T1 = [
    1, 2, 3;  % r1 -> e -> r2
    2, 1, 4;  % r2 -> w -> r1
    2, 3, 2;  % r2 -> s -> r3
    3, 2, 1;  % r3 -> n -> r2
    3, 7, 3;  % r3 -> e -> r7
    7, 3, 4;  % r7 -> w -> r3
    3, 4, 2;  % r3 -> s -> r4
    4, 3, 1;  % r4 -> n -> r3
    4, 5, 2;  % r4 -> s -> r5
    5, 4, 1;  % r5 -> n -> r4
    5, 6, 4;  % r5 -> w -> r6
    6, 5, 3;  % r6 -> e -> r5
];


% Definitions for Robot automaton G_R
E2 = ['n', 's', 'e', 'w', 'r'];  % Add event 'r' for rotation 90deg CW

X2 = [
    % State names               Corresponding Number
    'r1N'; 'r1S'; 'r1E'; 'r1W';  %  1,  2,  3,  4
    'r2N'; 'r2S'; 'r2E'; 'r2W';  %  5,  6,  7,  8
    'r3N'; 'r3S'; 'r3E'; 'r3W';  %  9,  10, 11, 12
    'r4N'; 'r4S'; 'r4E'; 'r4W';  %  13, 14, 15, 16
    'r5N'; 'r5S'; 'r5E'; 'r5W';  %  17, 18, 19, 20
    'r6N'; 'r6S'; 'r6E'; 'r6W';  %  21, 22, 23, 24
    'r7N'; 'r7S'; 'r7E'; 'r7W';  %  25, 26, 27, 28
];  

% Transitions for G_R
T2 = [
    1, 3, 5;  % r1N -> r -> r1E 
    3, 2, 5;  % r1E -> r -> r1S
    2, 4, 5;  % r1S -> r -> r1W
    4, 1, 5;  % r1W -> r -> r1N
    3, 7, 3;  % r1E -> e -> r2E

    5, 7, 5;  % r2N -> r -> r2E 
    7, 6, 5;  % r2E -> r -> r2S
    6, 8, 5;  % r2S -> r -> r2W
    8, 5, 5;  % r2W -> r -> r2N
    8, 4, 4;  % r2W -> w -> r1W
    6, 10, 2; % r2S -> s -> r3S

    9, 11, 5;   % r3N -> r -> r3E 
    11, 10, 5;  % r3E -> r -> r3S
    10, 12, 5;  % r3S -> r -> r3W
    12, 9, 5;   % r3W -> r -> r3N
    10, 14, 2;  % r3S -> s -> r4S
    9, 5, 1;    % r3N -> n -> r2N
    11, 27, 3;  % r3E -> e -> r7E

    13, 15, 5;  % r4N -> r -> r4E 
    15, 14, 5;  % r4E -> r -> r4S
    14, 16, 5;  % r4S -> r -> r4W
    16, 13, 5;  % r4W -> r -> r4N
    13, 9, 1;   % r4N -> n -> r3N
    14, 18, 2;  % r4S -> s -> r5S

    17, 19, 5;  % r5N -> r -> r5E 
    19, 18, 5;  % r5E -> r -> r5S
    18, 20, 5;  % r5S -> r -> r5W
    20, 17, 5;  % r5W -> r -> r5N
    17, 13, 1;  % r5N -> n -> r4N
    20, 24, 4;  % r5W -> w -> r6W

    21, 23, 5;  % r6N -> r -> r6E 
    23, 22, 5;  % r6E -> r -> r6S
    22, 24, 5;  % r6S -> r -> r6W
    24, 21, 5;  % r6W -> r -> r6N
    23, 19, 3;  % r6E -> e -> r5E

    25, 27, 5;  % r7N -> r -> r7E 
    27, 26, 5;  % r7E -> r -> r7S
    26, 28, 5;  % r7S -> r -> r7W
    28, 25, 5;  % r7W -> r -> r7N
    28, 12, 4;  % r7W -> w -> r3W
];

% A function to generate the parallel composition of two FDAs
function [X_comp, T_comp] = fda_parallel_comp(X1, X2, T1, T2)

    % Generate composite states by doing a 'cross-concatenation'
    [X1_grid, X2_grid] = meshgrid(1:size(X1,1), 1:size(X2,1));
    % X_comp has size 196 * 2
    X_comp = [X1_grid(:), X2_grid(:)];
    
    % Initialise empty composite transition matrix
    T_comp = [];
    
    % For each transition in T1
    for i = 1:size(T1,1)
        % Find the matching transition(s) in T2 with the same event
        matching_t2 = T2(T2(:,3) == T1(i,3), :);
        
        for j = 1:size(matching_t2,1)
            % Find composite state indices
            from_comp_idx = find(X_comp(:,1) == T1(i,1) & X_comp(:,2) == matching_t2(j,1));
            to_comp_idx = find(X_comp(:,1) == T1(i,2) & X_comp(:,2) == matching_t2(j,2));
            
            % Add new transition to composite transition matrix
            T_comp = [T_comp; from_comp_idx, to_comp_idx, T1(i,3)];
        end
    end
end

function [X_po, T_po, E_po] = convert_to_partially_observable(X_parallel, T_parallel, E_parallel)

    % Replace events n, s, e, w with m, this is done with what is essentially a grep command. 
    E_po = unique([E_parallel(E_parallel ~= 'n' & E_parallel ~= 's' & E_parallel ~= 'e' & E_parallel ~= 'w'), 'm']);
    
    % Make a copy of the original transition matrix (for simplicity)
    T_po = T_parallel;
    
    % Replace n, s, e, w with m in the transition matrix
    for i = 1:size(T_po, 1)
        if any(T_po(i, 3) == [find(E_parallel == 'n'), find(E_parallel == 's'), find(E_parallel == 'e'), find(E_parallel == 'w')])
            T_po(i, 3) = find(E_po == 'm'); % Replace with index of 'm'
        end
    end
    
    % States remain the same
    X_po = X_parallel;
end

% QUESTION 3 ============================================================

% Compute parallel composition
[X_parallel, T_parallel] = fda_parallel_comp(X1, X2, T1, T2);

% Generate state labels for parallel composition
state_labels = cell(size(X_parallel,1), 1);
for i = 1:size(X_parallel,1)
    state_labels{i} = [X1(X_parallel(i,1),:), '-', X2(X_parallel(i,2),:)];
end

% Display events
disp("Events of G_M||G_R:");
% TODO: change to acc calculate union?
disp(E2); % events are the union of both, in this case E1 subset of E2

% Display states
disp("States of G_M||G_R:");
disp(char(state_labels));

% Display transition map f(x, x', e)
disp("Transitions f(x, x', e) in G_M||G_R:");
disp('Cur. state    Next state    Event');
for i = 1:size(T_parallel,1)
    fprintf('%s \t-> \t%s : \t%c\n', ...
        state_labels{T_parallel(i,1)}, ...
        state_labels{T_parallel(i,2)}, ...
        E1(T_parallel(i,3)));
end

% QUESTION 4 ============================================================

% Convert {n, s, e, w} events to {m}
[X_po, T_po, E_po] = convert_to_partially_observable(X_parallel, T_parallel, E2); 

% Display events
disp("Events of Partially Observable G_N:");
disp(E_po); 

% Display states
disp("States of Partially Observable G_N:");
state_labels_po = cell(size(X_po,1), 1);
for i = 1:size(X_po,1)
    state_labels_po{i} = [X1(X_po(i,1),:), '-', X2(X_po(i,2),:)];
end
disp(char(state_labels_po));

% Display transitions
disp("Transitions f(x, x', e) in G_N:");
disp('Cur. state    Next state    Event');
for i = 1:size(T_po,1)
    fprintf('%s \t-> \t%s : \t%c\n', ...
        state_labels_po{T_po(i,1)}, ...
        state_labels_po{T_po(i,2)}, ...
        E_po(T_po(i,3)));
end
