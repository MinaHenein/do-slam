function [rWrapped] = wrapAxisAngle(r,limits)

rWrapped = r;

switch limits
	case 'pi'
		if abs(norm(r)) > pi
			rWrapped = wrapToPi(norm(r))*unit(r);
		end
	case '2pi'
		if norm(r) < 0 || norm(r) > 2*pi
			rWrapped = wrapTo2Pi(norm(r))*unit(r);
		end
	end

end %function
