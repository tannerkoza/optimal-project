function idx = resample(wP,N)

Q = cumsum(wP); % cumulative sum
t = rand(1,N+1);
T = sort(t);
T(N+1) = 1;
idx = zeros(N,1);
i = 1; 
j = 1;

while i<=N
    if T(i) < Q(j)
        idx(i) = j;
        i = i + 1;
    else
        j = j + 1;
    end
end
end