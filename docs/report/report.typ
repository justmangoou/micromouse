// Báo cáo dự án: robot vượt vật cản trên vi điều khiển STM32F103C8.
// Dịch:  typst compile docs/report/report.typ docs/report/report.pdf
// Chỗ tô vàng [CẦN ĐIỀN: ...] là thông tin không có trong mã nguồn. Điền hết trước khi nộp.

#let todo(body) = highlight(fill: rgb("#fde047"), extent: 1.5pt)[#text(weight: "bold")[[CẦN ĐIỀN: #body]]]

#set document(title: "Báo cáo dự án: robot vượt vật cản", author: "Nhóm thực hiện")
#set text(font: ("Times New Roman", "Libertinus Serif"), size: 12pt, lang: "vi")
#set par(justify: true, leading: 0.78em, spacing: 0.95em, first-line-indent: 1.0cm)
#set heading(numbering: "1.1.1")
#set math.equation(numbering: "(1)")

// ---- chương, mục ---------------------------------------------------------------------------------------------------
#show heading.where(level: 1): it => {
  pagebreak(weak: true)
  counter(figure.where(kind: image)).update(0)
  counter(figure.where(kind: table)).update(0)
  v(1.2cm)
  align(center)[
    #if it.numbering != none [
      #text(size: 14pt, weight: "bold")[CHƯƠNG #counter(heading).display()] \
      #v(4pt)
    ]
    #text(size: 16pt, weight: "bold")[#upper(it.body)]
  ]
  v(1.0cm)
}
#show heading.where(level: 2): it => { v(0.8em); text(size: 14pt, weight: "bold")[#counter(heading).display() #it.body]; v(0.3em) }
#show heading.where(level: 3): it => { v(0.5em); text(size: 13pt, weight: "bold", style: "italic")[#counter(heading).display() #it.body]; v(0.2em) }
#show heading: set par(first-line-indent: 0pt)

// ---- hình, bảng ----------------------------------------------------------------------------------------------------
#set figure(numbering: n => numbering("1.1", counter(heading).get().first(), n))
#show figure.where(kind: table): set figure.caption(position: top)
#show figure.where(kind: table): set figure(supplement: "Bảng")
#show figure.where(kind: image): set figure(supplement: "Hình")
#show figure.caption: set text(size: 12pt)
#show figure.caption: set par(first-line-indent: 0pt, justify: false)
#show figure: set block(above: 1.4em, below: 1.4em)
#show raw: set text(font: ("Menlo", "DejaVu Sans Mono"), size: 0.85em)
#show math.equation: set block(above: 1.0em, below: 1.0em)

#let front(title, brk: false) = {
  if brk { pagebreak(weak: true) }
  v(0.4cm)
  align(center, text(size: 14pt, weight: "bold")[#upper(title)])
  v(0.4cm)
}
#let rule = 0.6pt + black
#let tbl3(columns, align: auto, ..cells) = table(
  columns: columns, align: align, inset: (x: 8pt, y: 6pt), stroke: none,
  table.hline(stroke: rule), ..cells, table.hline(stroke: rule),
)
#let diagram-box(body, fill: luma(245)) = rect(width: 100%, inset: 9pt, stroke: 0.7pt + black, fill: fill)[#align(center)[#body]]

// ====================================================================================================================
// TRANG BÌA
// ====================================================================================================================
#page(numbering: none, margin: (left: 3.5cm, right: 2cm, y: 2.5cm))[
  #set par(first-line-indent: 0pt, justify: false)
  #align(center)[
    #text(size: 14pt, weight: "bold")[#upper[#todo[Tên trường]]] \
    #text(size: 14pt, weight: "bold")[#upper[#todo[Khoa / Bộ môn]]]
    #v(4pt)
    #line(length: 30%, stroke: 0.8pt)
    #v(2.2cm)
    #text(size: 16pt, weight: "bold")[BÁO CÁO DỰ ÁN]
    #v(0.4cm)
    #text(size: 14pt)[Môn học: *Nhập môn Hệ thống nhúng* \ Mã môn học: #todo[mã môn học]]
    #v(2.0cm)
    #text(size: 22pt, weight: "bold")[ROBOT DÒ ĐƯỜNG \ VƯỢT VẬT CẢN]
    #v(0.5cm)
    #text(size: 14pt, style: "italic")[Xây dựng trên vi điều khiển STM32F103C8 với cảm biến khoảng cách ToF và con quay hồi chuyển]
  ]
  #v(1fr)
  #align(left)[
    #pad(left: 3cm)[
      #table(
        columns: (auto, auto), stroke: none, inset: (x: 6pt, y: 5pt), align: (left, left),
        [Giảng viên hướng dẫn:], [#todo[họ tên, học hàm, học vị]],
        [Sinh viên thực hiện:], [#todo[họ tên các thành viên]],
        [Mã số sinh viên:], [#todo[MSSV]],
        [Nhóm:], [#todo[số nhóm]],
      )
    ]
  ]
  #v(1.2cm)
  #align(center)[#todo[Thành phố, tháng/năm nộp]]
]

// ====================================================================================================================
// PHẦN ĐẦU: số La Mã
// ====================================================================================================================
#set page(paper: "a4", margin: (left: 3.5cm, right: 2cm, top: 2.5cm, bottom: 2.5cm), numbering: "i", number-align: center)
#counter(page).update(1)

#front(brk: true)[Lời cam đoan]

Nhóm xin cam đoan nội dung báo cáo này là kết quả làm việc của nhóm trong quá trình thực hiện dự án môn học. Phần mềm của robot do nhóm tự
viết, trừ thư viện VL53L0X của hãng STMicroelectronics được dùng nguyên bản (xem tệp `NOTICE` trong thư mục dự án). Các tài liệu, số liệu
và thuật toán lấy từ nguồn khác đều được trích dẫn trong phần Tài liệu tham khảo.

#v(1.2cm)
#align(right)[
  #todo[Địa điểm, ngày tháng năm] \
  #v(4pt)
  *Đại diện nhóm* \
  #v(1.5cm)
  #todo[họ tên]
]

#v(0.8cm)
#front[Lời cảm ơn]

#todo[Lời cảm ơn giảng viên, bạn bè và những người đã hỗ trợ nhóm trong quá trình thực hiện dự án.]

#front(brk: true)[Tóm tắt]

Báo cáo trình bày việc thiết kế và chế tạo một robot hai bánh tự hành có khả năng chạy trên sa bàn kích thước 180 × 120 cm, trong đó có
17 vật cản đã biết vị trí, một vùng cấm và một vật cản ngẫu nhiên chỉ được đặt vào lúc thi. Toàn bộ chương trình chạy trên vi điều khiển
STM32F103C8 (72 MHz, RAM 20 KB, không có bộ tính dấu phẩy động). Robot ước lượng vị trí bằng cách kết hợp encoder bánh xe và con quay hồi
chuyển, lập đường đi bằng thuật toán A\* có xét khoảng cách tới vật cản, bám đường đi bằng thuật toán pure pursuit, và phát hiện vật cản
ngẫu nhiên bằng ba cảm biến khoảng cách VL53L0X so với dữ liệu nền thu từ các lượt chạy không có vật cản ngẫu nhiên. Khi phát hiện vật
cản, robot lập lại đường đi ngay trong lúc chạy. Một bộ giám sát dừng động cơ khi cảm biến hoặc thời gian thực thi có bất thường.

Phần điều hướng và điều khiển được kiểm thử bằng 40 ca kiểm thử trên máy tính, tất cả đều đạt. Chương trình sử dụng 84,0 % RAM và 82,5 %
bộ nhớ flash. Bốn lượt chạy được ghi lại trên sa bàn hoàn thành trong khoảng 8,6 đến 9,1 giây.

*Từ khoá:* robot tự hành, lập đường đi, A\*, pure pursuit, cảm biến ToF, STM32.

#front(brk: true)[Mục lục]
#set text(size: 11.5pt)
#outline(title: none, indent: 1.2em, depth: 2)

#front(brk: true)[Danh mục hình]
#outline(title: none, target: figure.where(kind: image))
#v(0.6cm)
#front[Danh mục bảng]
#outline(title: none, target: figure.where(kind: table))

#front(brk: true)[Danh mục từ viết tắt]
#set text(size: 12pt)

#tbl3((auto, 1fr), align: (left, left),
  [*Từ viết tắt*], [*Ý nghĩa*],
  [A\*], [Thuật toán tìm đường A-star],
  [FPU], [Bộ tính dấu phẩy động (floating-point unit)],
  [I²C], [Giao thức truyền thông nối tiếp hai dây],
  [IMU], [Khối đo quán tính (inertial measurement unit)],
  [PI], [Bộ điều khiển tỉ lệ - tích phân],
  [PWM], [Điều chế độ rộng xung],
  [SWD], [Giao diện gỡ lỗi nối tiếp (serial wire debug)],
  [ToF], [Cảm biến thời gian bay (time of flight)],
  [UART], [Giao tiếp nối tiếp bất đồng bộ],
)

// ====================================================================================================================
// PHẦN CHÍNH: số Ả Rập
// ====================================================================================================================
#set page(numbering: "1")
#counter(page).update(1)

= Giới thiệu

== Đặt vấn đề

Robot tự hành di chuyển trong môi trường có vật cản là bài toán điển hình của hệ thống nhúng: bộ xử lý phải đồng thời đọc cảm biến, ước
lượng trạng thái, ra quyết định và điều khiển cơ cấu chấp hành trong giới hạn thời gian thực và dung lượng bộ nhớ nhỏ. Trong môn học
Nhập môn Hệ thống nhúng, bài toán được cụ thể hoá thành một cuộc thi xe dò đường vượt vật cản trên sa bàn cố định.

Báo cáo này trình bày thiết kế của một robot hai bánh giải bài toán đó trên vi điều khiển STM32F103C8 (bo mạch Blue Pill). Đặc điểm của
vi điều khiển này là bộ nhớ rất hạn chế (RAM 20 KB, flash 64 KB) và không có bộ tính dấu phẩy động, nên mọi phép tính số thực đều phải
thực hiện bằng phần mềm. Điều đó buộc các thuật toán điều hướng phải được chọn và cài đặt cẩn thận về bộ nhớ và thời gian tính.

== Thể lệ cuộc thi

Theo thể lệ của môn học @rules, sa bàn rộng 180 cm, dài 120 cm. Băng dính đen dán thành đường chữ U; vùng giữa chữ U là vùng cấm, đi vào
đó được tính là ra khỏi sa bàn. Trên sa bàn có 17 khối vật cản kích thước 10 × 10 × 10 cm đặt cố định (ký hiệu T1 đến T17). Ở lượt thi thứ
hai, một khối ngẫu nhiên được đặt thêm vào một trong bốn vị trí đánh dấu sẵn ở phần bên phải sa bàn. Xe xuất phát sau vạch bên trái và
phải đưa cả thân xe qua vạch đích bên phải (@fig-arena).

#figure(
  image("fig/arena.png", width: 88%),
  caption: [Sa bàn thi: 17 vật cản tĩnh T1–T17, vùng xuất phát bên trái, vùng đích bên phải và vùng cấm ở giữa @rules.],
) <fig-arena>

Điểm chỉ được tính ở lượt thi thứ hai và do hệ thống chấm tự động (@tab-diem). Thể lệ nêu thứ tự ưu tiên: về đích và tránh vật cản, sau đó
đến không phạm lỗi, cuối cùng mới là tốc độ. Điểm tốc độ tối đa là 40, trong khi một vật cản bị xô lệch làm mất tổng cộng 15 điểm (5 điểm
phạt và 10 điểm của chính vật cản đó). Vì vậy nhóm chọn hướng thiết kế ưu tiên độ an toàn hơn tốc độ.

#figure(
  tbl3((1fr, auto), align: (left, right),
    [*Nội dung*], [*Điểm*],
    [Vượt một vật cản tĩnh], [+10],
    [Vượt vật cản ngẫu nhiên], [+15],
    [Về đích (cả thân xe qua vạch)], [+40],
    [Tốc độ, với $t$ là thời gian về đích (giây)], [(240 − $t$) / 6],
    [Chạy sạch (không lỗi, không chỉnh xe)], [+10],
    [Một bánh ra hẳn ngoài băng dính], [−5],
    [Xô lệch một vật cản], [−5],
    [Mỗi lần chỉnh xe], [−10],
  ),
  caption: [Cách tính điểm của lượt thi thứ hai @rules.],
) <tab-diem>

== Mục tiêu và phạm vi

Mục tiêu của dự án là chế tạo robot hoàn thành lượt thi ổn định, cụ thể gồm bốn yêu cầu: (1) về đích mà không chạm vật cản và không ra
ngoài băng dính; (2) phát hiện và vượt được khối ngẫu nhiên ở bất kỳ vị trí nào trong bốn vị trí; (3) chạy trong giới hạn bộ nhớ và tài
nguyên của vi điều khiển; (4) phần điều hướng có thể kiểm thử lặp lại trên máy tính. Tốc độ chỉ được tối ưu sau khi bốn yêu cầu trên đã đạt.

Phạm vi báo cáo gồm phần cứng của robot, các thuật toán định vị, điều khiển, lập đường đi và phát hiện vật cản, cùng kết quả kiểm thử.
Báo cáo không đề cập đến việc thiết kế khung cơ khí.

#todo[Nếu đề bài hoặc tiêu chí chấm thực tế của nhóm khác với thể lệ nêu trên, sửa lại mục này cho khớp.]

== Bố cục báo cáo

Chương 2 trình bày cơ sở lý thuyết của các thuật toán được dùng. Chương 3 mô tả phần cứng. Chương 4 mô tả thiết kế phần mềm và các tham
số cụ thể. Chương 5 trình bày kết quả kiểm thử và các lượt chạy thực tế. Chương 6 nêu kết luận, hạn chế và hướng phát triển.

= Cơ sở lý thuyết

== Ước lượng vị trí robot hai bánh vi sai <sec-odo>

Robot có hai bánh dẫn động độc lập với khoảng cách $b$ giữa hai bánh. Trong một chu kỳ $Delta t$, nếu hai bánh đi được các quãng đường
$Delta s_L$ và $Delta s_R$ (tính từ số xung encoder) thì quãng đường của tâm robot và tốc độ quay của hướng suy ra từ encoder là @siegwart

$ Delta s = (Delta s_L + Delta s_R) / 2, quad omega_e = (Delta s_R - Delta s_L) / (b thin Delta t). $

Vị trí $(x, y)$ và hướng $theta$ được cập nhật theo hướng ở giữa chu kỳ:

$ x <- x + Delta s cos(theta + omega Delta t / 2), quad y <- y + Delta s sin(theta + omega Delta t / 2), quad theta <- theta + omega Delta t. $

Sai số hướng của encoder cộng dồn nhanh khi bánh trượt. Con quay hồi chuyển đo trực tiếp tốc độ quay $omega_g$ và không bị ảnh hưởng
bởi trượt bánh, nhưng có độ lệch điểm không (bias) trôi theo nhiệt độ @mpu. Do đó tốc độ quay dùng để cập nhật hướng là tổ hợp của hai
nguồn (bộ lọc bù):

$ omega = alpha_g thin omega_g + (1 - alpha_g) thin omega_e, $

trong đó $alpha_g$ gần 1 để hướng chủ yếu lấy từ con quay.

== Điều khiển tốc độ bánh <sec-pi>

Mỗi bánh được điều khiển tốc độ bằng bộ điều khiển kết hợp bù trừ thuận (feed-forward) và PI. Nếu động cơ có đặc tính tuyến tính
$v = k (u - u_0)$ với $u$ là duty PWM, $k$ là hệ số tốc độ và $u_0$ là duty tối thiểu để bánh bắt đầu quay, thì duty cần đặt cho tốc độ
mong muốn $v^*$ là

$ u = v^* / k + u_0 + K_p e + K_i integral e thin dif t, quad e = v^* - v. $

Phần feed-forward $v^* / k + u_0$ đưa bánh đến gần tốc độ mong muốn, còn PI chỉ bù phần sai lệch còn lại. Phần tích phân được chặn
(anti-windup) để không tích luỹ quá mức khi đầu ra đã bão hoà.

== Thuật toán A\*

A\* tìm đường đi có chi phí nhỏ nhất trên đồ thị bằng cách mở rộng nút có giá trị $f(n) = g(n) + h(n)$ nhỏ nhất, trong đó $g(n)$ là chi phí đã
đi từ nút xuất phát và $h(n)$ là ước lượng chi phí còn lại tới đích @astar. Nếu $h$ không đánh giá quá chi phí thực thì kết quả là đường đi tối
ưu. Với lưới ô vuông 8 hướng, hàm ước lượng octile (đi thẳng có giá 10, đi chéo có giá 14) thường được dùng. Để đường đi tránh xa vật cản,
chi phí của mỗi bước có thể được nhân với một hệ số phạt tăng khi ô gần vật cản.

== Làm mượt đường đi bằng thuật toán Chaikin

Đường đi trên lưới gấp khúc theo ô nên không phù hợp để robot bám theo. Thuật toán cắt góc Chaikin thay mỗi đoạn thẳng $P_i P_(i+1)$ bằng
hai điểm nằm tại 1/4 và 3/4 đoạn, rồi lặp lại; sau vài vòng lặp đường gấp khúc trở thành đường cong trơn @chaikin.

== Thuật toán pure pursuit <sec-pp>

Pure pursuit là thuật toán bám đường đi hình học cho robot hai bánh @coulter. Ở mỗi bước, robot chọn trên đường đi một điểm cách nó một
khoảng nhìn trước $L$ rồi tính cung tròn đi qua điểm đó và đi tiếp theo hướng hiện tại. Gọi $alpha$ là góc giữa hướng robot và đường nối tới
điểm nhìn trước, $d$ là khoảng cách tới điểm đó (@fig-pp). Độ cong của cung tròn và tốc độ hai bánh là

$ kappa = (2 sin alpha) / d, quad v_L = v (1 - kappa b / 2), quad v_R = v (1 + kappa b / 2). $

Khoảng nhìn trước $L$ càng lớn thì robot đi càng mượt nhưng càng cắt góc; $L$ càng nhỏ thì bám sát hơn nhưng dễ dao động.

#figure(
  {
    box(width: 11cm, height: 7.4cm, {
      let rx = 3.2cm
      let ry = 6.6cm
      let tx = 7.4cm
      let ty = 1.4cm
      place(top + left, curve(
        stroke: (paint: luma(120), thickness: 1.8pt),
        curve.move((rx, ry)),
        curve.cubic((rx + 0.2cm, ry - 3.0cm), (tx - 2.2cm, ty + 1.2cm), (tx, ty)),
      ))
      place(top + left, line(start: (rx, ry), end: (rx, ry - 4.4cm), stroke: (paint: black, thickness: 1.2pt, dash: "dashed")))
      place(top + left, line(start: (rx, ry), end: (tx, ty), stroke: (paint: black, thickness: 1.2pt)))
      place(top + left, dx: rx - 0.45cm, dy: ry - 0.3cm, rect(width: 0.9cm, height: 1.1cm, fill: luma(225), stroke: 1pt))
      place(top + left, dx: tx - 3pt, dy: ty - 3pt, circle(radius: 3pt, fill: black, stroke: none))
      place(top + left, dx: tx + 6pt, dy: ty - 8pt, text(size: 10.5pt)[điểm nhìn trước])
      place(top + left, dx: rx + 0.15cm, dy: ry - 2.5cm, text(size: 12pt)[$alpha$])
      place(top + left, dx: rx + 2.1cm, dy: ry - 2.5cm, text(size: 12pt)[$d$])
      place(top + left, dx: rx - 0.7cm, dy: ry - 4.8cm, text(size: 10pt)[hướng robot])
      place(top + left, dx: 5.3cm, dy: 3.5cm, text(size: 10pt)[đường đi])
    })
  },
  kind: image,
  caption: [Sơ đồ hình học của thuật toán pure pursuit.],
) <fig-pp>

= Thiết kế phần cứng

== Các bộ phận chính

Robot gồm vi điều khiển, hai động cơ có encoder, mạch cầu H điều khiển động cơ, ba cảm biến khoảng cách, một con quay hồi chuyển và các
thiết bị giao diện đơn giản (@tab-hw). Vi điều khiển STM32F103C8 dùng lõi ARM Cortex-M3 tốc độ 72 MHz @stm32. Cảm biến khoảng cách
VL53L0X đo khoảng cách bằng thời gian bay của tia laser, giao tiếp qua bus I²C @vl53; MPU6050 là khối đo quán tính, trong đó chỉ dùng
con quay quanh trục thẳng đứng @mpu. Do ba cảm biến VL53L0X có cùng địa chỉ I²C mặc định, mỗi cảm biến có một chân tắt nguồn (XSHUT) riêng để
phần mềm bật lần lượt và gán địa chỉ khác nhau khi khởi động.

#figure(
  tbl3((auto, 1fr), align: (left, left),
    [*Bộ phận*], [*Thông số*],
    [Vi điều khiển], [STM32F103C8 (Blue Pill), 72 MHz, RAM 20 KB, flash 64 KB, không có FPU],
    [Động cơ và encoder], [Hai động cơ DC có encoder vuông pha, 270 xung mỗi vòng bánh (sau giải mã ×4), bánh đường kính 33 mm, khoảng cách hai bánh 82 mm],
    [Mạch điều khiển động cơ], [Mạch cầu H kiểu TB6612: hai chân chiều quay và một chân PWM cho mỗi động cơ],
    [Cảm biến khoảng cách], [3 cảm biến VL53L0X: phải 45°, phía trước, trái 45°],
    [Cảm biến quán tính], [MPU6050, dùng con quay trục z],
    [Giao diện], [Một nút bấm bắt đầu, một đèn LED trạng thái, UART chỉ truyền (115200 baud) để gỡ lỗi],
    [Nạp chương trình], [ST-Link qua giao diện SWD],
  ),
  caption: [Các bộ phận chính của robot (theo tệp `include/config.h`).],
) <tab-hw>

#todo[Model động cơ và tỉ số truyền, loại pin và điện áp, tên mạch điều khiển động cơ, kích thước và khối lượng robot.]

== Bố trí cảm biến khoảng cách

Ba cảm biến được gắn ở phía trước thân xe, hướng sang phải 45°, thẳng về phía trước và sang trái 45° (@fig-sensors). Hai cảm biến chéo
cho phép phát hiện vật cản ở hai bên đường đi, còn cảm biến thẳng phía trước phát hiện vật cản nằm ngay trên hướng chuyển động.

Mỗi cảm biến có sai số đo hệ thống nên được bù riêng. Nhóm đặt một tấm phẳng cách cảm biến 100 mm và đo (chương trình `test_tof_cal`),
thu được độ lệch bù +31 mm (phải), +38 mm (trước) và +34 mm (trái). Nếu không bù, cả ba cảm biến đều đọc thiếu khoảng 3 cm. Độ lệch bù gắn
với từng cảm biến vật lý nên phải đo lại khi thay cảm biến; nhóm đã phải làm vậy một lần với cảm biến phía trước.

#figure(
  {
    let s = 0.95pt
    box(width: 12cm, height: 7.0cm, {
      let cx = 6.0cm
      let cy = 4.7cm
      place(top + left, dx: cx - 50 * s, dy: cy - 62 * s, rect(width: 100 * s, height: 125 * s, stroke: 1.2pt, fill: luma(235)))
      place(top + left, dx: cx - 50 * s - 6pt, dy: cy - 10 * s, rect(width: 6pt, height: 33 * s, fill: black))
      place(top + left, dx: cx + 50 * s, dy: cy - 10 * s, rect(width: 6pt, height: 33 * s, fill: black))
      for (y, x, ang, lab) in (
        (-30, 60, -45, "phải 45°"),
        (0, 65, 0, "phía trước"),
        (30, 60, 45, "trái 45°"),
      ) {
        let px = cx - y * s
        let py = cy - x * s
        let len = 75 * s
        let dx = -calc.sin(ang * 1deg) * len
        let dy = -calc.cos(ang * 1deg) * len
        place(top + left, dx: px, dy: py, line(start: (0pt, 0pt), end: (dx, dy), stroke: (paint: black, thickness: 1.4pt, dash: "dashed")))
        place(top + left, dx: px + dx * 1.05 - 20pt, dy: py + dy * 1.05 - 14pt, text(size: 10.5pt)[#lab])
        place(top + left, dx: px - 2pt, dy: py - 2pt, circle(radius: 2pt, fill: black, stroke: none))
      }
      place(top + left, dx: cx - 18pt, dy: cy - 6pt, text(size: 10pt)[tâm xe])
      place(top + left, dx: cx - 34pt, dy: cy + 62 * s + 8pt, text(size: 9.5pt)[hướng tiến = lên trên])
    })
  },
  kind: image,
  caption: [Bố trí ba cảm biến khoảng cách, nhìn từ trên xuống, theo toạ độ lắp trong `include/config.h` (thân xe chỉ mang tính minh hoạ).],
) <fig-sensors>

== Tài nguyên của vi điều khiển

RAM 20 KB là tài nguyên hạn chế nhất. Bộ lập đường đi ở độ phân giải đầy đủ (ô lưới 20 mm) cần khoảng 46 KB, vượt quá dung lượng
RAM. Vì vậy robot dùng cấu hình tiết kiệm bộ nhớ (ô lưới 40 mm, bộ đệm đường đi ngắn hơn) chỉ cần khoảng 13 KB, vẫn giữ nguyên thuật toán.
Toàn bộ bộ nhớ của bộ lập đường đi là bộ nhớ tĩnh; chương trình không cấp phát động khi chạy. @tab-res cho kết quả dựng chương trình.

#figure(
  tbl3((1fr, auto, auto), align: (left, right, right),
    [*Tài nguyên*], [*Đã dùng*], [*Tỉ lệ*],
    [RAM (tổng 20 480 byte)], [17 212 byte], [84,0 %],
    [Flash (tổng 65 536 byte)], [54 072 byte], [82,5 %],
  ),
  caption: [Mức sử dụng bộ nhớ của chương trình (môi trường `course_nav`, PlatformIO).],
) <tab-res>

= Thiết kế phần mềm

== Kiến trúc tổng thể

Chương trình gồm hai vòng lặp chạy ở hai tốc độ (@fig-loops). Vòng nhanh, chạy theo ngắt bộ định thời TIM4 ở 200 Hz, chỉ có nhiệm vụ giữ cho
mỗi bánh quay đúng tốc độ được yêu cầu. Vòng chậm, chu kỳ 33 ms (khoảng 30 Hz), đọc cảm biến, ước lượng vị trí, phát hiện vật cản, lập đường
đi, tính tốc độ mong muốn của hai bánh và gửi cho vòng nhanh. Vòng chậm không điều khiển trực tiếp PWM. Nếu quá 150 ms không có lệnh mới,
động cơ tự tắt.

#figure(
  {
    grid(
      columns: (1fr, 0.12fr, 1fr), column-gutter: 4pt,
      [
        #align(center)[*Vòng chậm, 33 ms*]
        #v(2pt)
        #diagram-box[Đọc con quay, encoder, 3 cảm biến ToF]
        #align(center)[↓]
        #diagram-box[Ước lượng vị trí (odometry)]
        #align(center)[↓]
        #diagram-box[Phát hiện vật cản, lập đường đi, pure pursuit]
        #align(center)[↓]
        #diagram-box[Bộ giám sát an toàn]
      ],
      align(center + horizon)[#text(size: 20pt)[→] \ #text(size: 9pt)[tốc độ \ mong muốn]],
      [
        #align(center)[*Vòng nhanh, 5 ms*]
        #v(2pt)
        #diagram-box[Đo tốc độ từng bánh bằng encoder]
        #align(center)[↓]
        #diagram-box[Feed-forward + PI]
        #align(center)[↓]
        #diagram-box[Xuất PWM ra mạch cầu H]
        #v(8pt)
        #diagram-box(fill: luma(225))[Quá 150 ms không có lệnh mới: tắt động cơ]
      ],
    )
  },
  kind: image,
  caption: [Hai vòng lặp của phần mềm.],
) <fig-loops>

Mã nguồn được tổ chức để phần không phụ thuộc phần cứng (ước lượng vị trí, lập đường đi, bộ giám sát) nằm trong các thư viện `lib/base` và
`lib/course_nav`, nhờ đó biên dịch và kiểm thử được trên máy tính. Phần phụ thuộc phần cứng (động cơ, cảm biến, bộ định thời) nằm trong thư mục
`src/`. Thông số của robot được đặt ở `include/config.h`, thông số riêng của sa bàn ở `include/course_params.h`.

Một lượt chạy diễn ra như sau. Sau khi bật nguồn, robot đứng yên khoảng 4 giây để cảm biến khởi động và con quay được hiệu chuẩn. Người vận
hành bấm nút KEY, robot đếm ngược 3 giây rồi chạy tới vạch đích và tự dừng. Hai lượt chạy gần nhất được lưu vào bộ nhớ flash để đọc lại sau.

== Ước lượng vị trí

Phần ước lượng vị trí cài đặt đúng mô hình ở mục @sec-odo với chu kỳ 33 ms, khoảng cách hai bánh $b = 82$ mm và trọng số con quay $alpha_g = 0,98$. Nếu
tốc độ quay suy ra từ encoder và từ con quay chênh nhau hơn 1 rad/s thì xem như bánh đang trượt và chỉ dùng con quay.

Độ lệch điểm không của con quay MEMS trôi khi chip nóng lên; độ lệch 0,1°/s đã gây sai hướng 6° sau mỗi phút. Vì vậy chương trình đo độ lệch lúc
khởi động (2 giây chờ ổn định rồi lấy trung bình 2 giây; kết quả bị loại nếu các mẫu dao động quá 0,01 rad/s hoặc độ lệch vượt 0,1 rad/s) và
đo lại với hằng số thời gian 10 giây mỗi khi robot đứng yên với động cơ tắt ít nhất 0,5 giây.

== Điều khiển tốc độ bánh

Ở các phiên bản đầu, robot rung lắc, vọt lên khoảng 170 % tốc độ yêu cầu sau khi xuất phát, và có lần đứng yên gần một giây dù cả hai bánh đang
được lệnh chạy tới. Nhóm chạy thử nghiệm đáp ứng bậc thang (`src/test_step.cpp`) với các mức duty khác nhau và thấy nguyên nhân là mô hình động
cơ giả định ban đầu sai: động cơ thực nhanh hơn khoảng 4,5 lần. Bộ PI vì thế luôn bị bão hoà, và phần tích phân tích luỹ quá mức nên giữ đầu ra
ở mức âm cả giây sau khi mục tiêu đổi.

Sau khi đo lại, mỗi bánh dùng một mô hình riêng (@tab-motor) và bộ điều khiển ở mục @sec-pi với $K_p = 0,0006$ duty trên mm/s, $K_i = 0,003$ duty trên
mm/s trên giây, tổng phần hiệu chỉnh PI bị chặn trong ±0,3. Dưới 10 mm/s chỉ dùng feed-forward. Do một xung encoder trong 5 ms tương ứng 77 mm/s, tốc độ
được đo trên bốn chu kỳ liên tiếp (19 mm/s mỗi xung) rồi lọc thông thấp. Sai số tốc độ trên robot thực giảm từ khoảng 145 xuống 50–65 mm/s.

#figure(
  tbl3((1fr, auto, auto), align: (left, right, right),
    [*Hệ số đo trên sàn*], [*Bánh trái*], [*Bánh phải*],
    [$k$ (mm/s trên một đơn vị duty)], [2725], [2690],
    [$u_0$ (duty)], [0,068], [0,065],
  ),
  caption: [Mô hình động cơ đo được. Bánh phải nhanh hơn bánh trái khoảng 3 %.],
) <tab-motor>

== Lập đường đi

=== Lưới và hàm chi phí

Sa bàn được chia thành các ô 40 mm; với mỗi ô, khoảng cách $c$ tới vật cản gần nhất được tính sẵn. Biên đường đi bị loại với lề 70 mm (một bánh nằm cách
đường tâm 50 mm). A\* tìm đường từ ô xuất phát tới bất kỳ ô nào sau vạch đích ($y <= -120$ mm, $x$ từ 1260 đến 1750 mm); việc chọn đích là cả một đoạn
thay vì một điểm giúp đường đi đi thẳng xuống theo hành lang nào thuận lợi. Chi phí mỗi bước được nhân với hệ số phạt

$ m(c) = 1 + 6 thin t^2, quad t = max(0, (140 - c) / 140), $

với $c$ tính bằng mm, nên đường đi tự nhiên nằm giữa các khe và tránh xa vật cản. Lần tìm đầu yêu cầu khoảng cách an toàn 90 mm tới vật cản; nếu không có
đường thì thử lại với 80 mm rồi 45 mm để robot luồn qua được thay vì đứng yên.

=== Hậu xử lý

Đường đi từ A\* được xử lý qua ba bước. Thứ nhất, bỏ các điểm thừa chừng nào đường thẳng tắt vẫn giữ khoảng cách 110 mm tới mọi vật cản. Thứ hai, làm
mượt bằng thuật toán Chaikin hai vòng, chỉ chấp nhận nếu mọi điểm mới vẫn đủ an toàn. Thứ ba, kéo dài thêm 300 mm sau điểm cuối để robot chạy xuyên qua vạch
đích, rồi lấy mẫu lại mỗi 20 mm.

=== Hệ số tốc độ

Mỗi điểm trên đường đi được gán một hệ số tốc độ trong khoảng 0,35 đến 1, bằng tích của hai thành phần. Thành phần thứ nhất lấy giá trị kém hơn trong hai hệ
số "gần vật cản" và "gần băng dính": hệ số theo vật cản bằng 1 khi cách vật cản từ 130 mm trở lên và giảm tuyến tính còn 0,4 ở 95 mm; hệ số theo băng dính
nằm trong khoảng 0,5 đến 1 và giảm khi robot tiến gần băng dính. Thành phần thứ hai là hệ số khúc cua $1 / (1 + 1,75 thin Delta phi)$, với $Delta phi$ là
thay đổi hướng của đường đi trên năm điểm mỗi bên. Nhờ vậy khe rộng 200 mm giữa T16 và T17 được đi ở khoảng 250 mm/s; trước khi có quy tắc này robot vào khe đó
ở 480 mm/s và đã chạm T17. @fig-speedmap cho thấy tốc độ đo được dọc đường đi thực tế.

#figure(
  image("fig/speed_map.png", width: 92%),
  caption: [Đường đi của lượt chạy 10 tô màu theo tốc độ đo được. Robot chạy nhanh ở đoạn thoáng đầu tiên và chậm lại ở các khe hẹp.],
) <fig-speedmap>

=== Thời gian lập đường đi

Một lần lập đường đi mất khoảng 145 ms. Vi điều khiển chỉ có một lõi nên vòng điều khiển bị dừng trong thời gian đó. Để động cơ không tự tắt do quá 150 ms
không có lệnh mới, chương trình kéo dài thời hạn lệnh lên 600 ms ngay trước mỗi lần lập lại đường đi và cho phép bản chụp cảm biến kế tiếp cũ hơn bình thường.
Trước khi sửa, mỗi lần lập lại đường đi làm robot dừng khoảng 1,2 giây.

== Bám đường đi

Robot bám đường đi bằng pure pursuit (mục @sec-pp) với khoảng nhìn trước $L = 100 + 0,24 thin |v|$ (mm, với $v$ tính bằng mm/s). Nếu góc $alpha$ lớn hơn 1,2 rad, tức
điểm mục tiêu lệch hẳn sang bên hoặc ở phía sau, robot quay tại chỗ trước rồi mới đi tiếp. Tốc độ lệnh là

$ v = s thin V_"full" thin sqrt(f) thin max(0,35; cos alpha), $

trong đó $s = 0,95$ là mức chạy ổn định, $V_"full" = 622$ mm/s là thang tốc độ điều hướng (không phải tốc độ tối đa của động cơ) và $f$ là hệ số tốc độ của đường đi.
Ở phần trái và phần trên sa bàn, nơi mọi vật cản đều có trên bản đồ, $f$ không được thấp hơn 0,5. Ngoài ra có thêm các quy tắc: ở nửa bên phải ($x > 1050$ mm), nơi
khối ngẫu nhiên có thể xuất hiện, tốc độ nhân với 0,68; trong 0,8 giây sau khi phát hiện khối mới, tốc độ giảm một nửa; tốc độ không thấp hơn 22 % $V_"full"$ vì động
cơ sẽ không quay; ở đoạn cuối ($y < 250$ mm) tốc độ không thấp hơn 45 % để robot không bò qua vạch. Nếu robot cách đường đi hơn 250 mm, nó lập lại đường đi từ vị trí
hiện tại.

#todo[Tốc độ chạy ổn định 0,95, hệ số nhìn trước 0,24 và hệ số thận trọng 0,68 trong `course_params.h` chưa được commit; `docs/DECISIONS.md` vẫn ghi giá trị cũ (0,88; 0,22; 1,0). Commit giá trị cuối cùng hoặc ghi rõ các lượt chạy đã dùng giá trị nào.]

== Phát hiện vật cản ngẫu nhiên

=== Các phương án đã thử

Robot không biết khối ngẫu nhiên nằm ở đâu nên phải so số đo của cảm biến với số đo lẽ ra phải có. Phương án đầu tiên là so với khoảng cách dự đoán từ bản đồ 17 vật
cản. Phương án này báo sai rất nhiều vì bản đồ không có tường sa bàn, tường vùng cấm và băng dính đích: một lượt chạy không có khối thêm đã báo tám khối giả và lập
lại đường đi 36 lần (21,6 giây thay vì 9,9 giây). Việc thêm tường vào dự đoán và chỉ xét các điểm chạm nằm gần đường đi giảm được một phần nhưng chưa đủ.

=== Dữ liệu nền từ các lượt chạy sạch

Phương án cuối cùng dùng dữ liệu nền (baseline). Robot chạy vài lượt không có khối ngẫu nhiên; với mỗi ô 30 mm theo $(x, y)$ và mỗi khoảng 0,15 rad theo hướng, chương trình
`tools/make_baseline.py` lưu khoảng cách ngắn nhất mà mỗi cảm biến từng đo được. Tường, băng dính, vùng cấm hay một khối đặt lệch vài cm đều đã nằm trong dữ liệu
này nên không bị báo nhầm. Tệp `include/baseline.h` hiện có 80 ô, tạo từ 3 lượt chạy sạch.

=== Điều kiện kết luận có khối mới

Một số đo được coi là có khối mới khi thoả mãn đồng thời bốn điều kiện (@fig-detect): (1) ngắn hơn giá trị nền hơn 100 mm (ở nơi chưa có dữ liệu nền thì so với dự đoán từ bản đồ với
dung sai 40 mm cộng 8 % khoảng đo); số đo trên 800 mm hoặc dưới 40 mm bị bỏ qua; (2) kéo dài ba bước liên tiếp (0,1 giây); (3) điểm chạm nằm trong vòng 150 mm quanh đường
đi ở 700 mm phía trước và ở bên phải $x = 1150$ mm; (4) tâm ước lượng của khối nằm trong sa bàn. Tâm khối được ước lượng bằng mặt gần mà cảm biến thấy cộng nửa cạnh khối dọc
theo tia đo. Do ước lượng này có thể lệch tới 10 cm, bộ lập đường đi coi khối là một đĩa bán kính 110 mm (lớn hơn nửa cạnh thực 50 mm). Robot chỉ nhận tối đa một khối, đúng thể lệ,
và lập lại đường đi tối đa mỗi 0,2 giây.

#figure(
  {
    grid(
      columns: (1fr, 0.1fr, 1fr, 0.1fr, 1fr, 0.1fr, 1fr), column-gutter: 2pt, align: horizon,
      diagram-box[Số đo ngắn hơn dữ liệu nền hơn 100 mm?],
      align(center)[→],
      diagram-box[Kéo dài 3 bước liên tiếp (0,1 s)?],
      align(center)[→],
      diagram-box[Gần đường đi phía trước, ở nửa phải sa bàn?],
      align(center)[→],
      diagram-box(fill: luma(225))[Đặt đĩa Ø220 mm, lập lại đường đi],
    )
  },
  kind: image,
  caption: [Các điều kiện để robot kết luận có khối vật cản mới.],
) <fig-detect>

== An toàn và xử lý lỗi

Ở mỗi bước điều khiển, bộ giám sát (`lib/base/include/reliability.h`) quyết định robot có được phép chạy hay không. Bộ giám sát có năm trạng thái (@fig-states). Khi số đo
cảm biến cũ hoặc thiếu, bước điều khiển bị trễ, vị trí không hợp lệ hoặc không có đường đi, robot chuyển sang trạng thái Recovering: động cơ tắt và chỉ chạy tiếp sau khi robot đã đứng yên
và hoạt động bình thường liên tục 500 ms. Một số lỗi nặng được chốt lại cho tới khi khởi động lại.

#figure(
  {
    grid(
      columns: (1fr, 0.2fr, 1fr, 0.2fr, 1fr), row-gutter: 14pt, align: horizon,
      diagram-box[*Ready* \ chờ bấm KEY], align(center)[→], diagram-box[*Running* \ đang chạy], align(center)[→], diagram-box[*Done* \ đã về đích],
      [], [], align(center)[↕ \ #text(size: 9.5pt)[có lỗi / hết lỗi]], [], [],
      [], [], diagram-box(fill: luma(225))[*Recovering* \ tắt động cơ, chờ ổn định], [], diagram-box(fill: luma(225))[*Fault* \ lỗi nặng, chờ khởi động lại],
    )
  },
  kind: image,
  caption: [Các trạng thái của bộ giám sát (sơ đồ rút gọn).],
) <fig-states>

Robot được coi là về đích khi vị trí đã qua vạch đích ($y < -90$ mm) ở cột bên phải ($1215 < x < 1750$ mm) và đã đi được ít nhất 80 % đường đi. Khi có lỗi, đèn LED nháy số lần tương ứng với
mã lỗi (@tab-led). Hai lượt chạy gần nhất được lưu vào hai vùng 5 KB của flash, vùng cũ hơn bị ghi đè; mỗi bản ghi 16 byte ở tần số 30 Hz chứa thời gian, vị trí, góc lái, tốc độ mục tiêu
và tốc độ đo của hai bánh, cùng ba khoảng cách. Dữ liệu được đọc lại bằng `tools/read_run.py`, và các hình kết quả trong báo cáo đều vẽ từ nhật ký này.

#figure(
  tbl3((auto, 1fr, auto, 1fr), align: (center, left, center, left),
    [*Số lần nháy*], [*Lỗi*], [*Số lần nháy*], [*Lỗi*],
    [1], [Cảm biến], [5], [Vị trí],
    [2], [Thời gian], [6], [Cấu hình],
    [3], [Lệnh hết hạn], [7], [Quá 240 giây],
    [4], [Không có đường đi], [8], [Khởi động lại giữa lượt chạy],
  ),
  caption: [Mã lỗi hiển thị trên đèn LED (`src/course.cpp`).],
) <tab-led>

= Kiểm thử và kết quả

== Kiểm thử trên máy tính

Phần điều hướng và điều khiển được viết không phụ thuộc phần cứng nên biên dịch và kiểm thử được trên máy tính bằng khung kiểm thử Unity, một lần với bộ lập đường đi đầy đủ và một lần với
cấu hình tiết kiệm bộ nhớ đang chạy trên robot (`pio test -e native`, `pio test -e native_lowmem`). Kết quả của cấu hình thứ hai được nêu ở @tab-test: 40 ca kiểm thử, tất cả đều đạt.

#figure(
  tbl3((1fr, auto), align: (left, right),
    [*Bộ kiểm thử*], [*Số ca*],
    [Toán học (quy đổi góc, nội suy)], [9],
    [Bố trí cảm biến], [3],
    [Bộ giám sát độ tin cậy], [9],
    [Điều hướng (vị trí, PID, đường đi, phát hiện khối, chạy vòng kín)], [19],
    [*Tổng cộng*], [*40 (đạt 40)*],
  ),
  caption: [Kết quả kiểm thử trên máy tính (`pio test -e native_lowmem`).],
) <tab-test>

Các lỗi gặp trên robot thực đều được viết thành ca kiểm thử để không tái phát. Ví dụ, một bước rút gọn đường đi trong cấu hình tiết kiệm bộ nhớ từng nhảy thẳng tới điểm cuối mà không kiểm tra khoảng
cách, làm đường đi sượt T16 với một bánh cách 15 mm và robot đã chạm vào T16; ca `test_planned_path_keeps_clear_of_blocks` hiện kiểm tra điều này.

== Các lượt chạy trên sa bàn

Thư mục `data/` có bốn lượt chạy được đọc từ flash của robot (@tab-runs). Thời gian là mốc cuối cùng trong nhật ký; chiều dài đường đi được cộng từ các vị trí odometry trong nhật ký nên không phải số đo
trên sa bàn. Cột cuối chỉ áp công thức điểm tốc độ của thể lệ vào thời gian và không phải điểm thi thực tế.

#figure(
  tbl3((auto, auto, auto, auto, auto), align: (left, right, right, right, right),
    [*Lượt*], [*Thời gian* (s)], [*Đường đi* (mm)], [*Tốc độ đỉnh* (mm/s)], [*(240 − $t$)/6*],
    [1], [8,64], [2635], [632], [38,6],
    [2], [8,64], [2633], [632], [38,6],
    [9], [8,80], [2632], [618], [38,5],
    [10], [9,07], [2671], [610], [38,5],
  ),
  caption: [Bốn lượt chạy được ghi lại.],
) <tab-runs>

@fig-map là đường đi của hai lượt gần nhất dựng lại từ nhật ký. Robot đi giữa T1–T6 và vùng cấm, qua khe giữa T7, T9 và T11, rồi giữa T12 và T14 và chạy thẳng xuống ở cột bên phải.

#figure(
  image("fig/run_map.png", width: 95%),
  caption: [Đường đi của lượt 10 (nét liền) và lượt 9 (nét đứt), dựng lại từ nhật ký trong flash.],
) <fig-map>

@fig-signals là các tín hiệu của lượt 10. Tốc độ đo bám sát tốc độ yêu cầu: tăng lên khoảng 600 mm/s ở đoạn thoáng đầu tiên, giảm còn 200–300 mm/s ở các khe hẹp và giữ gần 400 mm/s ở đoạn thẳng cuối, nên tốc độ trung
bình chỉ khoảng 330 mm/s. Góc lái luôn trong khoảng ±0,8 rad, thấp hơn ngưỡng 1,2 rad để quay tại chỗ. Ở đồ thị cuối, giá trị 1200 mm nghĩa là cảm biến không thấy vật nào trong tầm đo.

#figure(
  image("fig/run10_signals.png", width: 88%),
  caption: [Lượt chạy 10: tốc độ bánh, góc lái và ba khoảng cách đo được.],
) <fig-signals>

== Kiểm tra phát hiện vật cản ngẫu nhiên

Việc phát hiện được kiểm tra theo ba cách (ghi trong `docs/DECISIONS.md`). Thứ nhất, lần lượt bỏ từng lượt chạy sạch ra khỏi dữ liệu nền rồi phát lại qua mã phát hiện bằng `tools/replay_run.cpp`; kết quả không có
khối giả nào. Thứ hai, một lượt có khối đặt ở cửa hành lang bên phải được phát lại và chạy thật; khối được tìm thấy trước khi chạm từ 0,1 đến 1 giây. Thứ ba, mô phỏng vòng kín trên máy tính với các khối chưa có
trên bản đồ.

#todo[Số lần thử với khối ngẫu nhiên ở mỗi trong bốn vị trí và số lần vượt không chạm. Repo không có số liệu tổng hợp này; nên lập bảng gồm vị trí, số lần thử, số lần vượt không chạm, số lần lập lại đường đi.]

#todo[Phiên bản phần mềm (commit) đã tạo ra các lượt chạy trên, và điểm thi chính thức hoặc điểm chạy thử nếu có.]

= Kết luận và hướng phát triển

== Kết luận

Dự án đã xây dựng được một robot tự hành hoàn chỉnh trên vi điều khiển STM32F103C8, gồm ước lượng vị trí từ encoder và con quay, lập đường đi A\* có xét khoảng cách an toàn, bám đường đi bằng pure
pursuit, phát hiện vật cản ngẫu nhiên bằng dữ liệu nền và lập lại đường đi trong lúc chạy, cùng bộ giám sát an toàn. Chương trình dùng 84 % RAM và 82,5 % flash. Phần điều hướng và điều khiển có 40 ca
kiểm thử trên máy tính, tất cả đều đạt, và các lượt chạy được ghi lại hoàn thành trong 8,6 đến 9,1 giây.

Có hai bài học chính. Thứ nhất, cần đo đặc tính động cơ thực thay vì giả định, vì mô hình sai là nguyên nhân của hiện tượng rung lắc và vọt tốc độ. Thứ hai, với môi trường lặp lại như sa bàn thi, việc học phần
nền từ các lượt chạy sạch hiệu quả hơn việc mô hình hoá thủ công từng bức tường.

#todo[Một hai câu về điều nhóm học được hoặc sẽ làm khác đi, bằng lời của nhóm.]

== Hạn chế

Vị trí của robot chỉ dựa vào encoder và con quay nên bị trôi vài cm trên cả sa bàn. Tính năng hiệu chỉnh vị trí theo bản đồ đã được viết và kiểm thử trên máy tính nhưng đang tắt, vì một số khối trên sa bàn
thực đặt lệch bản vẽ vài cm (T14 lệch 5 cm) và từng kéo vị trí robot lệch 5 cm. Dữ liệu nền phải được tạo lại từ vài lượt chạy sạch mỗi khi sa bàn hoặc cảm biến thay đổi. Khối nằm sau T14 bị phát hiện
muộn, và cảm biến phía trước đọc ngắn hơn thực tế 10–20 % ở khoảng cách 400–500 mm. Hệ số tỉ lệ của con quay chưa được hiệu chuẩn, mới chỉ hiệu chuẩn độ lệch điểm không. Cuối cùng, bộ nhớ đã dùng gần
hết nên còn rất ít chỗ để thêm tính năng.

== Hướng phát triển

Việc đáng làm trước hết là đo vị trí thực của 17 khối trên sa bàn rồi bật tính năng hiệu chỉnh vị trí theo bản đồ; khi vị trí chính xác hơn, robot có thể chạy nhanh hơn mà vẫn an toàn. Ngoài ra có thể
hiệu chuẩn hệ số tỉ lệ của con quay và chuyển sang vi điều khiển có bộ tính dấu phẩy động để rút ngắn thời gian lập đường đi (hiện khoảng 145 ms).

// ====================================================================================================================
#heading(numbering: none)[Tài liệu tham khảo]
#set par(first-line-indent: 0pt)
#bibliography("refs.yml", style: "ieee", title: none, full: true)

#todo[Kiểm tra lại cách trình bày tài liệu tham khảo theo mẫu mà trường yêu cầu; thêm các nguồn khác mà nhóm thực sự đã dùng (kể cả công cụ AI hỗ trợ, nếu môn học yêu cầu khai báo).]

// ====================================================================================================================
#heading(numbering: none)[Phụ lục: Dựng, kiểm thử và chạy]
#set par(first-line-indent: 1.0cm)

Mọi lệnh chạy từ thư mục gốc của dự án; cần cài PlatformIO. Lý do của từng tham số trong mã nguồn được ghi ở `docs/DECISIONS.md`.

```
pio run -e course_nav -t upload        # dựng và nạp cho Blue Pill qua ST-Link
pio test -e native                     # kiểm thử trên máy tính, bộ lập đường đi đầy đủ
pio test -e native_lowmem              # kiểm thử trên máy tính, cấu hình dùng trên robot
python3 tools/read_run.py --csv run    # đọc hai lượt chạy gần nhất từ flash của robot
python3 tools/make_baseline.py data/clean_runs/*.csv > include/baseline.h
```
