# Cách để hiện bầu trời trong gazebo:

- Mở file .world, kiếm tag ```<scence></scence>```
- Thêm vào:
```ruby
<sky>
    <clouds>
    <speed> 0 </speed>
    </clouds>
</sky>
```
- Ta được như sau:
```ruby
<scene>
    <ambient>0.4 0.4 0.4 1</ambient>
    <background>0.7 0.7 0.7 1</background>
    <shadows>true</shadows>

    <sky>
        <clouds>
        <speed> 0 </speed>
        </clouds>
    </sky>
</scene>
```

# Cách để thêm \<light\> vào trong gazebo:

