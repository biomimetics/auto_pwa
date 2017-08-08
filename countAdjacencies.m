function counts = countAdjacencies(indices,n_class)

counts = zeros(n_class, n_class);
edges = zeros(size(indices));

for i = 1:length(indices)
  if ~edges(i)
    dest = indices(i);
    for j = (i+1):length(indices)
      if edges(j)
        dest = edges(j);
        break
      elseif  indices(j) ~= indices(i)
        dest = indices(j);
        break
      end
    end
    edges(i:j) = dest;
    if dest ~= indices(i)
      counts(indices(i),indices(j)) = counts(indices(i),indices(j)) + j-i;
    end
  end
end