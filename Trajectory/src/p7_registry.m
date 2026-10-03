function value = p7_registry(action,key,value)
% P7_REGISTRY Keep each normal-mode Simulink run isolated by a unique key.
persistent sessions
if isempty(sessions), sessions=containers.Map('KeyType','char','ValueType','any'); end
switch action
    case 'set', sessions(key)=value;
    case 'get', value=sessions(key);
    case 'remove'
        if isKey(sessions,key), remove(sessions,key); end
        value=[];
    otherwise, error('P7:Registry','Unknown registry operation.');
end
end
