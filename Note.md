0. Initial place
    ```
    x = 1, y = 1, z = 0
    R = [
        1 0 0 0;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1
    ]
    ```

1. U'ya gore y ekseninde 2 birim hareket
    ```
    T1 = Translate(0, 2, 0)

    CurrentPosition = T1 * R
    CurrentPosition = [
        1 0 0 1;
        0 1 0 3;
        0 0 1 0;
        0 0 0 1
    ]
    ```

2. R'ye gore a ekseni etrafinda 30° donus
    ```
    T2 = RotAboutZ(pi/6)

    CurrentPosition = T1 * R * T2 
    CurrentPosition = [
        0.8660  -0.5    0       1;
        0.5     0.8660  0       3;
        0       0       1       0;
        0       0       0       1
    ]
    ```

3. U'ya gore x ekseninde 1 birim
    ```
    T3 = Translate(1, 0, 0)

    CurrentPosition = T3 * T1 * R * T2 
    CurrentPosition = [
        0.8660  -0.5    0       2;
        0.5     0.8660  0       3;
        0       0       1       0;
        0       0       0       1
    ]
    ```

4. R'ye gore a ekseni etrafinda -60° donus
    ```
    T4 = RotAboutZ(-pi/3)

    CurrentPosition = T3 * T1 * R * T2 * T4
    CurrentPosition = [
        0.8660  0.5     0       2;
        -0.5    0.8660  0       3;
        0       0       1       0;
        0       0       0       1
    ]
    ```

5. U'ya gore y ekseninde -1 birim hareket
    ```
    T5 = Translate(0, -1, 0)

    CurrentPosition = T5 * T3 * T1 * R * T2 * T4
    CurrentPosition = [
        0.8660  0.5     0       2;
        -0.5    0.8660  0       2;
        0       0       1       0;
        0       0       0       1
    ]
    ```
6. R'ye gore a ekseni ektrafinda -60° donus
    ```
    T6 = RotAboutZ(-pi/3)

    CurrentPosition = T5 * T3 * T1 * R * T2 * T4 * T6
    CurrentPosition = [
        0       1       0       2;
        -1      0       0       2;
        0       0       1       0;
        0       0       0       1
    ]
    ```
7. R'ye gore n ekseninde 1 birim hareket
    ```
    T7 = Translate(1, 0, 0)

    CurrentPosition = T5 * T3 * T1 * R * T2 * T4 * T6 * T7
    CurrentPosition = [
        0       1       0       2;
        -1      0       0       1;
        0       0       1       0;
        0       0       0       1
    ]
    ```
8.  U'ya gore z ekseni etrafinda 30° donus
    ```
    T8 = RotAboutZ(pi/6)

    CurrentPosition = T8 * T5 * T3 * T1 * R * T2 * T4 * T6 * T7
    CurrentPosition = [
        0.5     0.8660  0       1.2321;
        -0.8660 0.5     0       1.8660;
        0       0       1       0;
        0       0       0       1
    ]
    ```


dünya referans çerçevesine göre delta operatörü ve çerçeve arasındaki ilişki nedir?

delta operatürü çerçeve ile çarptığımızda çerçeveyi değiştiriyor

dünya referans çerçevesine göre değişim için baştan çarpıyoruz
mevcüt çerçevesine göre aynı değişimi bulmak için delta operatörü sondan çarpıyoruz