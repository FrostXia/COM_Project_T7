function value = simulation_registry(action,key,value)
% SIMULATION_REGISTRY Store simulation sessions using numeric run IDs only.
persistent sessions nextID
if isempty(sessions), sessions=containers.Map('KeyType','double','ValueType','any'); end
if isempty(nextID), nextID=0; end
if strcmp(action,'register')
    nextID=nextID+1;
    assert(nextID<flintmax,'Trajectory:Registry','Run ID space exhausted.');
    sessions(nextID)=value;
    value=nextID;
    return
end
validateattributes(key,{'double'},{'scalar','real','finite','integer','positive'});
switch action
    case 'get'
        assert(isKey(sessions,key),'Trajectory:MissingSession', ...
            'No active run for this ID. Start the simulation from the app.');
        value=sessions(key);
    case 'remove'
        if isKey(sessions,key), remove(sessions,key); end
        value=[];
    otherwise
        error('Trajectory:Registry','Use register, get or remove.');
end
end
