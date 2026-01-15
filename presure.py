def PT(altitude):
    h = 1.01325 * (1 - ((0.0065*altitude)/288.15))**5.225
    t = 288.15 - 0.0065 * altitude
    return h, t 

h1, t1 = PT(10000)

print(f"h1 = {h1} bar = {h1*(10e5)} Pa")
print(f"t1 = {t1} °K = {t1-273.15} °C")