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
    '1N'; '1S'; '1E'; '1W';  %  1,  2,  3,  4
    '2N'; '2S'; '2E'; '2W';  %  5,  6,  7,  8
    '3N'; '3S'; '3E'; '3W';  %  9,  10, 11, 12
    '4N'; '4S'; '4E'; '4W';  %  13, 14, 15, 16
    '5N'; '5S'; '5E'; '5W';  %  17, 18, 19, 20
    '6N'; '6S'; '6E'; '6W';  %  21, 22, 23, 24
    '7N'; '7S'; '7E'; '7W';  %  25, 26, 27, 28
];  

% Transitions for G_R
T2 = [
    1, 3, 5;  % 1N -> r -> 1E 
    3, 2, 5;  % 1E -> r -> 1S
    2, 4, 5;  % 1S -> r -> 1W
    4, 1, 5;  % 1W -> r -> 1N
    3, 7, 3;  % 1E -> e -> 2E

    5, 7, 5;  % 2N -> r -> 2E 
    7, 6, 5;  % 2E -> r -> 2S
    6, 8, 5;  % 2S -> r -> 2W
    8, 5, 5;  % 2W -> r -> 2N
    8, 4, 4;  % 2W -> w -> 1W
    6, 10, 2; % 2S -> s -> 3S

    9, 11, 5;   % 3N -> r -> 3E 
    11, 10, 5;  % 3E -> r -> 3S
    10, 12, 5;  % 3S -> r -> 3W
    12, 9, 5;   % 3W -> r -> 3N
    10, 14, 2;  % 3S -> s -> 4S
    9, 5, 1;    % 3N -> n -> 2N
    11, 27, 3;  % 3E -> e -> 7E

    13, 15, 5;  % 4N -> r -> 4E 
    15, 14, 5;  % 4E -> r -> 4S
    14, 16, 5;  % 4S -> r -> 4W
    16, 13, 5;  % 4W -> r -> 4N
    13, 9, 1;   % 4N -> n -> 3N
    14, 18, 2;  % 4S -> s -> 5S

    17, 19, 5;  % 5N -> r -> 5E 
    19, 18, 5;  % 5E -> r -> 5S
    18, 20, 5;  % 5S -> r -> 5W
    20, 17, 5;  % 5W -> r -> 5N
    17, 13, 1;  % 5N -> n -> 4N
    20, 24, 4;  % 5W -> w -> 6W

    21, 23, 5;  % 6N -> r -> 6E 
    23, 22, 5;  % 6E -> r -> 6S
    22, 24, 5;  % 6S -> r -> 6W
    24, 21, 5;  % 6W -> r -> 6N
    23, 19, 3;  % 6E -> e -> 5E

    25, 27, 5;  % 7N -> r -> 7E 
    27, 26, 5;  % 7E -> r -> 7S
    26, 28, 5;  % 7S -> r -> 7W
    28, 25, 5;  % 7W -> r -> 7N
    28, 12, 4;  % 7W -> w -> 3W
];

function [X_comp, T_comp] = fda_parallel_comp(X1, X2, T1, T2)
    % Create composite states
    [X1_grid, X2_grid] = meshgrid(1:size(X1,1), 1:size(X2,1));
    X_comp = [X1_grid(:), X2_grid(:)];
    
    % Initialize empty transition matrix
    T_comp = [];
    
    % For each transition in T1
    for i = 1:size(T1,1)
        % Find matching transitions in T2 with same event
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

% Compute parallel composition
[X_parallel, T_parallel] = parallel_composition(X1, X2, T1, T2);

% Generate state labels for parallel composition
state_labels = cell(size(X_parallel,1), 1);
for i = 1:size(X_parallel,1)
    state_labels{i} = [X1(X_parallel(i,1),:), '-', X2(X_parallel(i,2),:)];
end

% Display results
disp('Events in GM||GR:');
disp(E1);

disp('States in GM||GR:');
disp(char(state_labels));

disp('Transitions in GM||GR:');
disp('From State    To State    Event');
for i = 1:size(T_parallel,1)
    fprintf('%s -> %s : %c\n', ...
        state_labels{T_parallel(i,1)}, ...
        state_labels{T_parallel(i,2)}, ...
        E1(T_parallel(i,3)));
end
