function d = mahalMetric(e,Ez)

if size(Ez,1) ~= size(Ez,2)
    Ez = cov(Ez',1);
end

d = sqrt(sum(e.* (Ez^-1 * e),1));