import { Dictionary } from '@/types/common'

class PullTracker {
  private layer_status: Dictionary<string> = {}

  // We need to assume a size for the missing layers, we are assuming 100mb for now.
  private guesstimated_size = 100 * 1024 * 1024

  private layers: string[] = []

  private layer_progress: Dictionary<string> = {}

  private layer_progress_detail: Dictionary<{total: number, current: number}> = {}

  private left_over_data = ''

  pull_output = ''

  private onready: () => void

  overall_status = ''

  download_percentage = 0

  extraction_percentage = 0

  constructor(ready_callback: () => void) {
    this.onready = ready_callback
  }

  updateSimplifiedProgress(): void {
    let download_total = 0
    let download_current = 0
    let extraction_total = 0
    let extraction_current = 0
    for (const id of Object.keys(this.layer_status)) {
      if (this.layer_status[id] === 'Already exists') {
        continue
      }
      switch (this.layer_status[id]) {
        case 'Waiting':
          download_total += this.guesstimated_size
          extraction_total += this.guesstimated_size
          break

        case 'Downloading':
          download_total += this.layer_progress_detail[id].total
          download_current += this.layer_progress_detail[id].current
          extraction_total += this.layer_progress_detail[id].total
          break

        case 'Pull complete':
          download_total += this.layer_progress_detail[id].total
          download_current += this.layer_progress_detail[id].total
          extraction_total += this.layer_progress_detail[id].total
          extraction_current += this.layer_progress_detail[id].total
          break

        case 'Download complete':
          download_total += this.layer_progress_detail[id]?.total ?? 0
          download_current += this.layer_progress_detail[id]?.total ?? 0
          extraction_total += this.layer_progress_detail[id]?.total ?? 0
          break

        case 'Extracting':
          download_total += this.layer_progress_detail[id].total
          download_current += this.layer_progress_detail[id].total
          extraction_total += this.layer_progress_detail[id].total
          extraction_current += this.layer_progress_detail[id].current
          break
        default:
          // some data is not consumed
          break
      }
    }
    this.download_percentage = download_current / download_total / 0.01
    this.extraction_percentage = extraction_current / extraction_total / 0.01
  }

  digestNewData(progressEvent: {currentTarget: { response: string}}): void {
    // dataChunk contains the data that have been obtained so far (the whole data so far)..
    // The received data is descbribed at
    // https://docker-py.readthedocs.io/en/stable/api.html#docker.api.image.ImageApiMixin.pull
    const dataChunk = progressEvent?.currentTarget?.response
    // As the data consists of multiple jsons, the following like is a hack to split them
    const dataList = (this.left_over_data + dataChunk.replaceAll('}{', '}\n\n{')).split('\n\n')
    this.left_over_data = ''

    for (const line of dataList) {
      try {
        const data = JSON.parse(line)
        if ('id' in data) {
          const { id } = data
          if (!this.layers.includes(id)) {
            this.layers.push(id)
          }
          if ('progress' in data) {
            this.layer_progress[id] = data.progress
          }
          if ('status' in data) {
            this.layer_status[id] = data.status
          }
          if (data?.progressDetail?.total !== undefined) {
            this.layer_progress_detail[id] = data.progressDetail
          }
        } else {
          this.overall_status = data.status
          // Axios returns the promise too early (before the pull is done)
          // so we check the overall docker status instead
          if (this.overall_status.includes('Downloaded newer image for')) {
            this.onready()
          }
          if (this.overall_status.includes('Image is up to date')) {
            this.onready()
          }
        }
      } catch (error) {
        this.left_over_data = line
      }
    }
    this.pull_output = ''
    this.layers.forEach((image) => {
      this.pull_output = `${this.pull_output}[${image}] ${this.layer_status[image]}`
        + `  ${this.layer_progress[image] || ''}\n`
    })
    this.pull_output = `${this.pull_output}${this.overall_status}\n`

    this.updateSimplifiedProgress()
  }
}
export default PullTracker
