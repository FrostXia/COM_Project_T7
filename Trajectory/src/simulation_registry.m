function value = simulation_registry(action,key,value)
% SIMULATION_REGISTRY Isolate runs using numeric IDs; accept legacy character keys.
persistent sessions nextID
if isempty(sessions)
    sessions=containers.Map('KeyType','char','ValueType','any');
end
if isempty(nextID), nextID=0; end
if strcmp(action,'register')
    nextID=nextID+1;
    assert(nextID<flintmax,'Trajectory:Registry','Run ID space exhausted.');
    sessions(sprintf('run_%d',nextID))=value;
    value=nextID;
    return
end
if isnumeric(key)
    validateattributes(key,{'double'},{'scalar','real','finite','integer','positive'});
    key=sprintf('run_%d',key);
else
    validateattributes(key,{'char'},{'row','nonempty'});
end
switch action
    case 'set', sessions(key)=value;
    case 'get'
        assert(isKey(sessions,key),'Trajectory:MissingSession', ...
            'No active run for this ID. Start the simulation from the Trajectory app.');
        value=sessions(key);
    case 'remove'
        if isKey(sessions,key), remove(sessions,key); end
        value=[];
    otherwise, error('Trajectory:Registry','Unknown registry operation.');
end
end
