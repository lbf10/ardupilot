y = [2 3 5 4 6];
k = [0:4]';
polyn = @(x)[ones(length(x),1) x x.^2 x.^3];
X = polyn(k);
theta = [inv(X'*X)*X'*y']';

curve = [];
for it=0:0.1:4
    curve = [curve polyn(it)*theta'];
end
plot(0:0.1:4,curve)
hold on
plot(k,y,'r')

dpolyn = @(x)theta(2)+2*theta(3)*x+3*theta(4)*x^2;

curve = [];
for it=0:0.1:4
    curve = [curve dpolyn(it)];
end
figure
plot(0:0.1:4,curve)
hold on
plot(1:4,diff(y))