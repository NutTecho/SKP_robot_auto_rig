require 'sketchup.rb'

module RobotCommonMultiStudio
  @dialog = nil
  @selected_robot_name = nil
  @robots_data = {}

  # แก้ไขบรรทัดล่างนี้บรรทัดเดียว
  DICT_NAME = "RobotUniversalData" unless defined?(DICT_NAME)

  def self.make_unique_deep(entity)
    return unless entity.is_a?(Sketchup::Group) || entity.is_a?(Sketchup::ComponentInstance)
    entity.make_unique if entity.respond_to?(:make_unique)

    sub_ents = entity.is_a?(Sketchup::Group) ? entity.entities : entity.definition.entities
    sub_ents.each do |child|
      make_unique_deep(child) if child.is_a?(Sketchup::Group) || child.is_a?(Sketchup::ComponentInstance)
    end
  end

  def self.get_robot_root(robot_name)
    model = Sketchup.active_model
    model.entities.find do |e|
      (e.is_a?(Sketchup::Group) || e.is_a?(Sketchup::ComponentInstance)) && e.name.strip == robot_name
    end
  end

  def self.scan_all_robots
    model = Sketchup.active_model
    names = []

    model.entities.each do |e|
      if e.is_a?(Sketchup::Group) || e.is_a?(Sketchup::ComponentInstance)
        has_j1 = false
        walker = lambda do |sub_ents|
          sub_ents.each do |sub|
            if sub.is_a?(Sketchup::Group) || sub.is_a?(Sketchup::ComponentInstance)
              has_j1 = true if ['Base', 'J1'].include?(sub.name.strip)
              walker.call(sub.definition.entities) if sub.is_a?(Sketchup::Group)
            end
          end
        end
        ents = e.is_a?(Sketchup::Group) ? e.entities : e.definition.entities
        walker.call(ents)

        if has_j1
          n = e.name.strip.empty? ? "Robot_#{e.entityID}" : e.name.strip
          e.name = n if e.name.strip.empty?
          names << n
        end
      end
    end
    names.uniq
  end

  def self.find_robot_parts(robot_name)
    root = get_robot_root(robot_name)
    return {} unless root

    dict = {}
    walker = lambda do |entities|
      entities.each do |e|
        if e.is_a?(Sketchup::Group) || e.is_a?(Sketchup::ComponentInstance)
          name = e.name.strip
          dict[name] = e if ['Base', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6'].include?(name)
          sub_ents = e.is_a?(Sketchup::Group) ? e.entities : e.definition.entities
          walker.call(sub_ents)
        end
      end
    end
    ents = root.is_a?(Sketchup::Group) ? root.entities : root.definition.entities
    walker.call(ents)
    dict
  end

  def self.solve_joint_center(parent_ent, child_ent)
    bp = parent_ent.bounds
    bc = child_ent.bounds
    ix1 = [bp.min.x, bc.min.x].max; ix2 = [bp.max.x, bc.max.x].min
    iy1 = [bp.min.y, bc.min.y].max; iy2 = [bp.max.y, bc.max.y].min
    iz1 = [bp.min.z, bc.min.z].max; iz2 = [bp.max.z, bc.max.z].min

    if ix1 <= ix2 && iy1 <= iy2 && iz1 <= iz2
      Geom::Point3d.new((ix1 + ix2) / 2.0, (iy1 + iy2) / 2.0, (iz1 + iz2) / 2.0)
    else
      Geom::Point3d.new((bp.center.x + bc.center.x) / 2.0, (bp.center.y + bc.center.y) / 2.0, (bp.max.z + bc.min.z) / 2.0)
    end
  end

  def self.find_manual_pivot(part, fallback_axis, fallback_pt)
    return [fallback_pt, fallback_axis] unless part && part.valid?
    
    best_pt = fallback_pt
    best_axis = fallback_axis

    walker = lambda do |ents, tr|
      ents.each do |e|
        if (e.is_a?(Sketchup::Group) || e.is_a?(Sketchup::ComponentInstance))
          name = e.name.strip.downcase
          if name.start_with?('pivot') || name.start_with?('center')
            best_pt = e.bounds.center.transform(tr)
            
            if name.include?('-x')
              best_axis = Geom::Vector3d.new(-1, 0, 0)
            elsif name.include?('x')
              best_axis = Geom::Vector3d.new(1, 0, 0)
            elsif name.include?('-y')
              best_axis = Geom::Vector3d.new(0, -1, 0)
            elsif name.include?('y')
              best_axis = Geom::Vector3d.new(0, 1, 0)
            elsif name.include?('-z')
              best_axis = Geom::Vector3d.new(0, 0, -1)
            elsif name.include?('z')
              best_axis = Geom::Vector3d.new(0, 0, 1)
            end
            
            return true
          end
        end
      end
      
      ents.each do |e|
        if e.is_a?(Sketchup::Group)
          return true if walker.call(e.entities, tr * e.transformation)
        elsif e.is_a?(Sketchup::ComponentInstance)
          return true if walker.call(e.definition.entities, tr * e.transformation)
        end
      end
      false
    end
    
    ents = part.is_a?(Sketchup::Group) ? part.entities : part.definition.entities
    walker.call(ents, part.transformation)
    
    [best_pt, best_axis]
  end

  def self.setup_kinematics(robot_name, preset_key)
    @selected_robot_name = robot_name
    model = Sketchup.active_model

    root = get_robot_root(robot_name)
    return { success: false, message: "ไม่พบ Group หุ่นยนต์: #{robot_name}" } unless root

    make_unique_deep(root)
    parts = find_robot_parts(robot_name)

    is_scara = (preset_key == 'scara')
    required = is_scara ? ['Base', 'J1', 'J2', 'J3'] : ['Base', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6']
    missing = required.reject { |k| parts.key?(k) }

    if missing.any?
      return { success: false, message: "[#{robot_name}] ขาดชิ้นส่วน: #{missing.join(', ')}" }
    end

    model.start_operation("Rig Universal #{robot_name}", true)

    pivots = {}
    axes_dirs = {}

    bb_base = parts['Base'].bounds
    b_j1    = parts['J1'].bounds
    b_j2    = parts['J2'].bounds
    b_j3    = parts['J3'].bounds

    if is_scara
      inter_x = [bb_base.min.x, b_j1.min.x].max + [bb_base.max.x, b_j1.max.x].min
      inter_y = [bb_base.min.y, b_j1.min.y].max + [bb_base.max.y, b_j1.max.y].min
      
      fb_pt1 = Geom::Point3d.new(inter_x / 2.0, inter_y / 2.0, b_j1.min.z)
      fb_ax1 = Geom::Vector3d.new(0, 0, 1)
      pivots['J1'], axes_dirs['J1'] = find_manual_pivot(parts['J1'], fb_ax1, fb_pt1)

      fb_pt2 = solve_joint_center(parts['J1'], parts['J2'])
      fb_ax2 = Geom::Vector3d.new(0, 0, 1)
      pivots['J2'], axes_dirs['J2'] = find_manual_pivot(parts['J2'], fb_ax2, fb_pt2)

      fb_pt3 = Geom::Point3d.new(b_j3.center.x, b_j3.center.y, b_j3.center.z)
      fb_ax3 = Geom::Vector3d.new(0, 0, -1)
      pivots['J3'], axes_dirs['J3'] = find_manual_pivot(parts['J3'], fb_ax3, fb_pt3)

      pivots['J4'] = pivots['J3']
      axes_dirs['J4'] = Geom::Vector3d.new(0, 0, 1)
      axes_list = ['J1', 'J2', 'J3', 'J4']
    else
      b_j4 = parts['J4'].bounds
      b_j5 = parts['J5'].bounds
      b_j6 = parts['J6'].bounds

      fb_pt1 = Geom::Point3d.new(b_j1.center.x, b_j1.center.y, b_j1.min.z)
      fb_ax1 = Geom::Vector3d.new(0, 0, 1)
      pivots['J1'], axes_dirs['J1'] = find_manual_pivot(parts['J1'], fb_ax1, fb_pt1)

      fb_pt2 = solve_joint_center(parts['J1'], parts['J2'])
      fb_ax2 = Geom::Vector3d.new(0, 1, 0)
      pivots['J2'], axes_dirs['J2'] = find_manual_pivot(parts['J2'], fb_ax2, fb_pt2)

      raw_j3 = solve_joint_center(parts['J2'], parts['J3'])
      fb_pt3 = Geom::Point3d.new(raw_j3.x, pivots['J2'].y, raw_j3.z)
      fb_ax3 = Geom::Vector3d.new(0, 1, 0)
      pivots['J3'], axes_dirs['J3'] = find_manual_pivot(parts['J3'], fb_ax3, fb_pt3)

      fb_pt4 = Geom::Point3d.new(b_j4.center.x, pivots['J3'].y, pivots['J3'].z)
      fb_ax4 = Geom::Vector3d.new(1, 0, 0)
      pivots['J4'], axes_dirs['J4'] = find_manual_pivot(parts['J4'], fb_ax4, fb_pt4)

      raw_j5 = solve_joint_center(parts['J4'], parts['J5'])
      fb_pt5 = Geom::Point3d.new(raw_j5.x, pivots['J4'].y, raw_j5.z)
      fb_ax5 = Geom::Vector3d.new(0, 1, 0)
      pivots['J5'], axes_dirs['J5'] = find_manual_pivot(parts['J5'], fb_ax5, fb_pt5)

      fb_pt6 = b_j6.center
      fb_ax6 = Geom::Vector3d.new(1, 0, 0)
      pivots['J6'], axes_dirs['J6'] = find_manual_pivot(parts['J6'], fb_ax6, fb_pt6)

      axes_list = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
    end

    model.active_entities.grep(Sketchup::ConstructionPoint).each(&:erase!)
    pivots.each { |_, pt| model.active_entities.add_cpoint(pt) }

    saved_angles = {}
    axes_list.each do |j|
      saved_angles[j] = root.get_attribute(DICT_NAME, j, 0.0).to_f
    end

    root.set_attribute(DICT_NAME, "preset", preset_key)

    @robots_data[robot_name] = {
      pivots: pivots,
      axes_dirs: axes_dirs,
      angles: saved_angles,
      preset: preset_key,
      is_scara: is_scara
    }

    model.commit_operation
    { success: true, message: "Rig [#{robot_name}] สำเร็จพร้อมใช้งาน!" }
  end

  def self.apply_jog(joint_name, target_val)
    return unless @selected_robot_name && @robots_data[@selected_robot_name]

    robot = @robots_data[@selected_robot_name]
    parts = find_robot_parts(@selected_robot_name)
    return unless robot[:pivots][joint_name]

    current_val = robot[:angles][joint_name] || 0.0
    delta = target_val - current_val
    return if delta.abs < 0.001

    model = Sketchup.active_model
    pivot = robot[:pivots][joint_name]
    axis  = robot[:axes_dirs][joint_name]

    if robot[:is_scara]
      case joint_name
      when 'J1'
        tr = Geom::Transformation.rotation(pivot, axis, delta.degrees)
        ['J1', 'J2', 'J3'].each do |p|
          parts[p].transform!(tr) if parts[p] && parts[p].valid?
          robot[:pivots][p].transform!(tr) if robot[:pivots][p] && p != 'J1'
        end
      when 'J2'
        tr = Geom::Transformation.rotation(pivot, axis, delta.degrees)
        ['J2', 'J3'].each do |p|
          parts[p].transform!(tr) if parts[p] && parts[p].valid?
          robot[:pivots][p].transform!(tr) if robot[:pivots][p] && p != 'J2'
        end
      when 'J3'
        down_vector = Geom::Vector3d.new(0, 0, -delta.mm)
        tr = Geom::Transformation.translation(down_vector)
        if parts['J3'] && parts['J3'].valid?
          parts['J3'].transform!(tr)
          robot[:pivots]['J3'].transform!(tr) if robot[:pivots]['J3']
          robot[:pivots]['J4'].transform!(tr) if robot[:pivots]['J4']
        end
      when 'J4'
        tr = Geom::Transformation.rotation(robot[:pivots]['J3'], axis, delta.degrees)
        parts['J3'].transform!(tr) if parts['J3'] && parts['J3'].valid?
      end
    else
      chain = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
      idx = chain.index(joint_name)
      return unless idx

      affected = chain[idx..-1]
      tr = Geom::Transformation.rotation(pivot, axis, delta.degrees)

      affected.each do |p_name|
        ent = parts[p_name]
        next unless ent && ent.valid?
        ent.transform!(tr)

        if p_name != joint_name && robot[:pivots][p_name]
          robot[:pivots][p_name].transform!(tr)
        end
        if p_name != joint_name && robot[:axes_dirs][p_name]
          robot[:axes_dirs][p_name].transform!(tr)
        end
      end
    end

    robot[:angles][joint_name] = target_val
    root = get_robot_root(@selected_robot_name)
    root.set_attribute(DICT_NAME, joint_name, target_val) if root

    model.active_view.invalidate
  end

  def self.show_ui
    if @dialog && @dialog.visible?
      @dialog.bring_to_front
      return
    end

    @dialog = UI::HtmlDialog.new({
      :dialog_title => "Universal Robot Studio",
      :preferences_key => "universal_robot_studio_v9",
      :width => 480,
      :height => 840,
      :resizable => true
    })

    html = <<-HTML
    <!DOCTYPE html>
    <html>
    <head>
      <meta charset="utf-8">
      <style>
        body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; padding: 14px; background: #f0f2f5; margin: 0; font-size: 13px; color: #333; }
        .tab-box { display: flex; margin-bottom: 12px; }
        .tab { flex: 1; text-align: center; padding: 10px; cursor: pointer; background: #dfe4ea; border-radius: 6px; margin: 0 2px; font-weight: bold; color: #57606f; }
        .tab.active { background: #2f3542; color: white; }
        .panel { display: none; }
        .panel.active { display: block; }
        .card { background: white; padding: 12px 14px; border-radius: 8px; margin-bottom: 10px; box-shadow: 0 1px 3px rgba(0,0,0,0.08); }
        .row { display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px; }
        label { font-weight: 600; color: #2f3542; }
        select, input[type=number].cfg-input { padding: 6px 8px; border-radius: 5px; border: 1px solid #ced6e0; width: 180px; font-size: 12px; }
        input[type=range] { width: 100%; margin: 6px 0; accent-color: #2ed573; }
        .angle-input { width: 60px; padding: 4px 6px; border: 1px solid #ced6e0; border-radius: 4px; text-align: right; font-family: monospace; font-size: 13px; font-weight: bold; }
        button { width: 100%; padding: 11px; border: none; border-radius: 6px; font-weight: bold; cursor: pointer; transition: 0.2s; }
        .btn-rig { background: #2ed573; color: white; font-size: 14px; margin-top: 6px; }
        .btn-rig:hover { background: #26af5f; }
        .btn-scan { background: #1e90ff; color: white; padding: 6px 12px; font-size: 12px; width: auto; }
        .btn-reset { background: #747d8c; color: white; margin-top: 8px; }
        #popup-status { display: none; padding: 10px; border-radius: 6px; margin-bottom: 10px; text-align: center; font-weight: bold; }
        .success { background: #d4edda; color: #155724; border: 1px solid #c3e6cb; }
        .error { background: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }
        
        .guide-box { background: #fdfdfd; border-left: 4px solid #ff9f43; padding: 10px 12px; margin-bottom: 12px; border-radius: 6px; font-size: 12px; box-shadow: 0 1px 2px rgba(0,0,0,0.05); }
        .guide-title { font-weight: bold; color: #d35400; display: flex; justify-content: space-between; align-items: center; cursor: pointer; }
        .guide-content { display: block; margin-top: 8px; line-height: 1.5; color: #57606f; }
        .code-tag { background: #eef1f6; color: #d63031; font-family: monospace; padding: 2px 5px; border-radius: 4px; font-weight: bold; border: 1px solid #dcdde1; }
      </style>
    </head>
    <body>
      <div id="popup-status"></div>

      <div class="card" style="border-left: 4px solid #1e90ff;">
        <div class="row">
          <label>Selected Robot</label>
          <div style="display:flex; gap:6px;">
            <select id="robot-select" onchange="onRobotChanged()"></select>
            <button class="btn-scan" onclick="refreshRobots()">Scan</button>
          </div>
        </div>
      </div>

      <div class="tab-box">
        <div class="tab active" onclick="switchTab('rig-panel')">1. Auto-Rig Setup</div>
        <div class="tab" onclick="switchTab('jog-panel')">2. Jog Controls</div>
      </div>

      <div id="rig-panel" class="panel active">
        <div class="guide-box">
          <div class="guide-title" onclick="toggleGuide()">
            <span>📖 คู่มือการตั้งชื่อ & บังคับแกนหมุน (Pivot Override)</span>
            <span id="guide-arrow" style="font-size:10px;">▲ ย่อ</span>
          </div>
          <div id="guide-body" class="guide-content">
            <b>1. การตั้งชื่อหุ่นยนต์ (Group นอกสุด):</b><br>
            ตั้งชื่อเป็นอะไรก็ได้ที่สื่อความหมาย เช่น <span class="code-tag">Robot_1</span>, <span class="code-tag">Nachi_MZ07</span>, <span class="code-tag">Mitsubishi_8CR</span><br><br>
            <b>2. การตั้งชื่อชิ้นส่วนภายใน:</b><br>
            <ul>
              <li><b>6 แกน:</b> <span class="code-tag">Base</span>, <span class="code-tag">J1</span>, <span class="code-tag">J2</span>, <span class="code-tag">J3</span>, <span class="code-tag">J4</span>, <span class="code-tag">J5</span>, <span class="code-tag">J6</span></li>
              <li><b>SCARA (4 แกน):</b> ตั้ง 4 ชิ้นคือ <span class="code-tag">Base</span>, <span class="code-tag">J1</span>, <span class="code-tag">J2</span>, และ <span class="code-tag">J3</span> (เพลา Ball Screw)</li>
            </ul>            <b>3. หากแกนไหนหมุนเยื้อง (เช่น หน้าแปลนชี้ลงพื้น):</b><br>
            เข้าไปใน Group นั้น -> สร้างวงกลมตรงจุดหมุน -> Make Group -> <b>ตั้งชื่อตามทิศทางที่แกนชี้ไป:</b><br>
            • <span class="code-tag">pivot_x</span> : ให้แกนหมุนรอบแกนสีแดง (เช่น J4, J6)<br>
            • <span class="code-tag">pivot_y</span> : ให้แกนหมุนรอบแกนสีเขียว (เช่น J2, J3, J5)<br>
            • <span class="code-tag">pivot_z</span> : ให้แกนหมุนรอบแกนสีน้ำเงิน (เช่น J1 หรือหน้าแปลนที่คว่ำลงพื้น)<br>
            • <span class="code-tag">-x, -y, -z</span> : ใส่เครื่องหมายลบด้านหน้าเพื่อกลับทิศทางการหมุน
          </div>
        </div>
        
        <div class="card">
          <div class="row">
            <label>Kinematics Type</label>
            <select id="preset-key" onchange="changeType()">
              <option value="6axis">6-Axis Articulated (Universal)</option>
              <option value="scara">4-Axis SCARA</option>
            </select>
          </div>
          <div class="row" id="row-stroke" style="display:none; margin-top:8px;">
            <label>Max Z-Stroke Length</label>
            <div>
              <input type="number" id="cfg-stroke" class="cfg-input" style="width:70px; text-align:right;" value="200" onchange="updateStrokeMax()">
              <span style="font-weight:bold; color:#747d8c;"> mm</span>
            </div>
          </div>
          <button class="btn-rig" onclick="triggerRig()">Auto-Rig & Calibrate</button>
        </div>
      </div>

      <div id="jog-panel" class="panel">
        <div id="sliders"></div>
        <button class="btn-reset" onclick="resetAll()">Reset to Zero (Home)</button>
      </div>

      <script>
        let currentRobot = '';
        let currentPreset = '6axis';
        let strokeMaxVal = 200;

        const config6 = [
          { id: 'J1', desc: 'J1 (Base Twist)', min: -170, max: 170, unit: '°', step: 0.5 },
          { id: 'J2', desc: 'J2 (Shoulder Pitch)', min: -110, max: 130, unit: '°', step: 0.5 },
          { id: 'J3', desc: 'J3 (Elbow Pitch)', min: -140, max: 150, unit: '°', step: 0.5 },
          { id: 'J4', desc: 'J4 (Forearm Roll)', min: -200, max: 200, unit: '°', step: 0.5 },
          { id: 'J5', desc: 'J5 (Wrist Pitch)', min: -120, max: 120, unit: '°', step: 0.5 },
          { id: 'J6', desc: 'J6 (Tool Flange)', min: -360, max: 360, unit: '°', step: 1.0 }
        ];

        function getScaraConfig() {
          return [
            { id: 'J1', desc: 'J1 (Arm 1 Rotation)', min: -170, max: 170, unit: '°', step: 0.5 },
            { id: 'J2', desc: 'J2 (Arm 2 Rotation)', min: -150, max: 150, unit: '°', step: 0.5 },
            { id: 'J3', desc: 'J3 (Z-Stroke Down)', min: 0, max: strokeMaxVal, unit: ' mm', step: 1.0 },
            { id: 'J4', desc: 'J4 (Tool Roll)', min: -360, max: 360, unit: '°', step: 1.0 }
          ];
        }

        function switchTab(id) {
          document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
          document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
          event.target.classList.add('active');
          document.getElementById(id).classList.add('active');
        }

        function toggleGuide() {
          const body = document.getElementById('guide-body');
          const arrow = document.getElementById('guide-arrow');
          if (body.style.display === 'block') {
            body.style.display = 'none';
            arrow.innerText = '▼ ขยาย';
          } else {
            body.style.display = 'block';
            arrow.innerText = '▲ ย่อ';
          }
        }

        function showPopup(text, isSuccess) {
          const p = document.getElementById('popup-status');
          p.className = isSuccess ? 'success' : 'error';
          p.innerText = text;
          p.style.display = 'block';
          setTimeout(() => { p.style.display = 'none'; }, 4000);
        }

        function refreshRobots() {
          sketchup.scanRobots();
        }

        function updateRobotDropdown(names) {
          const sel = document.getElementById('robot-select');
          sel.innerHTML = '';
          names.forEach(name => {
            const opt = document.createElement('option');
            opt.value = name;
            opt.innerText = name;
            sel.appendChild(opt);
          });
          if (names.length > 0) {
            currentRobot = sel.value;
            sketchup.switchRobot(currentRobot);
          }
        }

        function onRobotChanged() {
          currentRobot = document.getElementById('robot-select').value;
          sketchup.switchRobot(currentRobot);
        }

        function changeType() {
          currentPreset = document.getElementById('preset-key').value;
          document.getElementById('row-stroke').style.display = (currentPreset === 'scara') ? 'flex' : 'none';
          renderSliders();
        }

        function updateStrokeMax() {
          strokeMaxVal = parseFloat(document.getElementById('cfg-stroke').value) || 200;
          renderSliders();
        }

        function applyLoadedState(preset, savedAngles) {
          if (preset) {
            currentPreset = preset;
            document.getElementById('preset-key').value = preset;
            document.getElementById('row-stroke').style.display = (currentPreset === 'scara') ? 'flex' : 'none';
          }
          renderSliders(savedAngles || {});
        }

        function renderSliders(savedAngles = {}) {
          const list = document.getElementById('sliders');
          list.innerHTML = '';
          const defs = (currentPreset === 'scara') ? getScaraConfig() : config6;

          defs.forEach(j => {
            const rawVal = (savedAngles && savedAngles[j.id] !== undefined) ? savedAngles[j.id] : 0.0;
            const currentVal = parseFloat(rawVal) || 0.0;

            const card = document.createElement('div');
            card.className = 'card';
            card.innerHTML = `
              <div class="row">
                <label>${j.desc}</label>
                <div>
                  <input type="number" class="angle-input" id="num-${j.id}" min="${j.min}" max="${j.max}" step="${j.step}" value="${currentVal.toFixed(1)}">
                  <span style="font-weight:bold; color:#747d8c;">${j.unit}</span>
                </div>
              </div>
              <input type="range" id="rng-${j.id}" min="${j.min}" max="${j.max}" value="${currentVal}" step="${j.step}">
            `;
            list.appendChild(card);

            const rng = card.querySelector('input[type=range]');
            const num = card.querySelector('input[type=number]');

            rng.addEventListener('input', (e) => {
              const val = parseFloat(e.target.value);
              num.value = val.toFixed(1);
              sketchup.jogAxis(j.id, val);
            });

            num.addEventListener('change', (e) => {
              let val = parseFloat(e.target.value) || 0.0;
              if (val < j.min) val = j.min;
              if (val > j.max) val = j.max;
              num.value = val.toFixed(1);
              rng.value = val;
              sketchup.jogAxis(j.id, val);
            });
          });
        }

        function triggerRig() {
          currentRobot = document.getElementById('robot-select').value;
          currentPreset = document.getElementById('preset-key').value;
          if (!currentRobot) {
            showPopup("กรุณากด Scan แล้วเลือกหุ่นยนต์ก่อนครับ", false);
            return;
          }
          sketchup.execRig(currentRobot, currentPreset);
        }

        function onRigCompleted(res) {
          showPopup(res.message, res.success);
          if (res.success) {
            sketchup.switchRobot(currentRobot);
            setTimeout(() => {
              document.querySelectorAll('.tab')[1].click();
            }, 800);
          }
        }

        function resetAll() {
          const defs = (currentPreset === 'scara') ? getScaraConfig() : config6;
          defs.forEach(j => {
            const rng = document.getElementById('rng-' + j.id);
            const num = document.getElementById('num-' + j.id);
            if (rng && num) {
              rng.value = 0;
              num.value = '0.0';
              sketchup.jogAxis(j.id, 0);
            }
          });
        }

        renderSliders();
        setTimeout(refreshRobots, 300);
      </script>
    </body>
    </html>
    HTML

    @dialog.set_html(html)

    @dialog.add_action_callback("scanRobots") do |action_context|
      robots = scan_all_robots
      json_arr = "[" + robots.map { |r| "'#{r}'" }.join(',') + "]"
      @dialog.execute_script("updateRobotDropdown(#{json_arr});")
    end

    @dialog.add_action_callback("switchRobot") do |action_context, name|
      @selected_robot_name = name.to_s
      root = get_robot_root(@selected_robot_name)
      angles = {}
      preset = '6axis'

      if root
        preset = root.get_attribute(DICT_NAME, "preset", "6axis")
        axes = (preset == 'scara') ? ['J1', 'J2', 'J3', 'J4'] : ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
        axes.each do |axis|
          angles[axis] = root.get_attribute(DICT_NAME, axis, 0.0).to_f
        end
      end

      angles_json = "{" + angles.map { |k, v| "'#{k}': #{v}" }.join(',') + "}"
      @dialog.execute_script("applyLoadedState('#{preset}', #{angles_json});")
    end

    @dialog.add_action_callback("execRig") do |action_context, name, preset|
      res = setup_kinematics(name.to_s, preset.to_s)
      json_res = "{ success: #{res[:success]}, message: '#{res[:message]}' }"
      @dialog.execute_script("onRigCompleted(#{json_res});")
    end

    @dialog.add_action_callback("jogAxis") do |action_context, axis, val|
      apply_jog(axis.to_s, val.to_f)
    end

    @dialog.show
  end

  unless file_loaded?(__FILE__)
    UI.menu('Extensions').add_item('Universal Robot Studio') { show_ui }
    file_loaded?(__FILE__)
  end
end