function ones_block = gen_ones_block(N,b_length)


ones_block = zeros(N, numel(b_length));
depth = 1;
column = 1;
for i = 1:numel(b_length)
    temp = ones(b_length(i),1); % Creates a vector of ones
    ones_block(depth:depth+b_length(i)-1,column:column) = temp; % Inserts vector in the block
    depth = depth + b_length(i); % Increase depth
    column = column + 1; % Go to next column
end