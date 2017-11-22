function [new_value, index] = getValue(matrix, nom_cost, gamma)

% get the values surrounding the center (i,j) value of the matrix
north_val = matrix(1,2);
east_val = matrix(2,3);
south_val = matrix(3,2);
west_val = matrix(2,1);

% if we go north
V_north = north_val * (0.8) + east_val * (0.1) + west_val * (0.1) + nom_cost;

% east
V_east = east_val * (0.8) + south_val * (0.1) + north_val * (0.1) + nom_cost;

% south
V_south = south_val * (0.8) + west_val * (0.1) + east_val * (0.1) + nom_cost;

% west
V_west = west_val * (0.8) + south_val * (0.1) + north_val * (0.1) + nom_cost;

V_possible = [V_north V_east V_south V_west];

[val, idx] = max(V_possible);

new_value = gamma * val;
index = idx;


end