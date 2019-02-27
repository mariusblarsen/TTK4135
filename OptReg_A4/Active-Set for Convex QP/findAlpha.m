function alpha = findAlpha(A, x, b, p, k)
    alpha = 1;
    n = size(A,2);
    %n = 5;
    for i = 1:1:n
        disp("k = " + i);
        num = b(i,1)-A(:,i)'*x(:,k)
        denum = A(:,i)'*p(:,k)
        if (num/denum < alpha)
            alpha = num/denum;
        end
    end
end

    