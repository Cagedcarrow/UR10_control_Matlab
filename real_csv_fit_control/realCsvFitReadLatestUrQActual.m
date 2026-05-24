function [qLatest, rxBuffer, parsedPackets, expectedPacketLen, lostSync, bytesReadNow] = ...
    realCsvFitReadLatestUrQActual(tcpObj, rxBuffer, expectedPacketLen)
qLatest = [];
parsedPackets = 0;
lostSync = false;
bytesReadNow = 0;

avail = tcpObj.NumBytesAvailable;
if avail > 0
    newBytes = read(tcpObj, avail, 'uint8');
    rxBuffer = [rxBuffer; newBytes(:)];
    bytesReadNow = numel(newBytes);
end

validLens = [1220,1116,1108];
while numel(rxBuffer) >= 4
    if expectedPacketLen <= 0
        [syncIdx, foundLen] = localFindSyncHeader(rxBuffer, validLens);
        if syncIdx < 1
            if numel(rxBuffer) > 3
                rxBuffer = rxBuffer(end-2:end);
            end
            break;
        end
        if syncIdx > 1
            rxBuffer = rxBuffer(syncIdx:end);
        end
        expectedPacketLen = foundLen;
    end

    packetLen = localBeInt32(rxBuffer(1:4));
    if packetLen ~= expectedPacketLen
        rxBuffer = rxBuffer(2:end);
        expectedPacketLen = 0;
        lostSync = true;
        continue;
    end

    if numel(rxBuffer) < packetLen
        break;
    end

    packet = rxBuffer(1:packetLen);
    rxBuffer = rxBuffer(packetLen+1:end);
    qCand = localParseQActual(packet);
    if localIsValidQ(qCand)
        parsedPackets = parsedPackets + 1;
        qLatest = qCand;
    else
        expectedPacketLen = 0;
        lostSync = true;
    end
end
end

function [idx, foundLen] = localFindSyncHeader(rxBuffer, validLens)
idx = -1;
foundLen = 0;
n = numel(rxBuffer);
for k = 1:(n-3)
    L = localBeInt32(rxBuffer(k:k+3));
    if any(L == validLens) && n-k+1 >= L
        qCand = localParseQActual(rxBuffer(k:k+L-1));
        if localIsValidQ(qCand)
            idx = k;
            foundLen = L;
            return;
        end
    end
end
end

function q = localParseQActual(packet)
qOffset = 253;
q = zeros(1,6);
for i = 1:6
    i0 = qOffset + (i-1)*8;
    q(i) = localBeDouble(packet(i0:(i0+7)));
end
end

function tf = localIsValidQ(q)
tf = isvector(q) && numel(q) == 6 && all(isfinite(q)) && all(abs(q) < 20);
end

function v = localBeInt32(b)
v = double(b(1))*16777216 + double(b(2))*65536 + double(b(3))*256 + double(b(4));
end

function d = localBeDouble(b)
d = typecast(uint8(b(end:-1:1)), 'double');
end
